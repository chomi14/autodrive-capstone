"""Portable session audit, review journal and immutable dataset backend (no ROS runtime)."""
import collections
import hashlib
import json
import os
from pathlib import Path
import random
import re
import shutil
import sqlite3
import time
import uuid

import cv2
import numpy as np

SCHEMA = 'normal-e2e-v1'


def read_json(path):
    return json.loads(Path(path).read_text())


def save_json(path, data):
    path = Path(path)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(data, indent=2, allow_nan=False))
    os.replace(temp, path)


def digest(data):
    return hashlib.sha256(json.dumps(data, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def jsonl(path):
    rows, invalid = [], 0
    if Path(path).exists():
        for line in Path(path).read_text().splitlines():
            try: rows.append(json.loads(line))
            except ValueError: invalid += 1
    return rows, invalid


def stamp(msg):
    return int(msg.sec) * 1_000_000_000 + int(msg.nanosec)


def typestore():
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    store = get_typestore(Stores.ROS2_HUMBLE)
    defs = {}
    for file in (Path(__file__).parent / 'rosdefs').glob('*.msg'):
        defs.update(get_types_from_msg(file.read_text(), 'interfaces_pkg/msg/' + file.stem))
    store.register(defs)
    return store


def command_dict(m):
    return dict(command_id=m.command_id, camera_instance=m.camera_instance, source_sequence=int(m.source_sequence),
        source_stamp_ns=stamp(m.source_stamp), source_clock_domain=m.source_clock_domain,
        generated_ros_ns=int(m.generated_ros_ns), processing_started_monotonic_ns=int(m.processing_started_monotonic_ns),
        completed_monotonic_ns=int(m.completed_monotonic_ns), steering=int(m.command.steering),
        left_pwm=int(m.command.left_speed), right_pwm=int(m.command.right_speed),path_valid=bool(m.path_valid),
        lane_visible=bool(m.lane_visible),target_lane=int(m.target_lane), controller_enabled=bool(m.controller_enabled),state=m.state)


def match(frames, commands, transmissions):
    """Only identity+original integer stamp+domain. No timestamp-nearest fallback."""
    counts = collections.Counter()
    frame_map, command_map = collections.defaultdict(list), collections.defaultdict(list)
    for f in frames: frame_map[(f['camera_instance'], f['source_sequence'])].append(f)
    for c in commands: command_map[c['command_id']].append(c)
    tx_map = collections.defaultdict(list)
    for t in transmissions: tx_map[t.get('command_id')].append(t)
    conflicting_sources=set()
    by_source = collections.defaultdict(list)
    for cid, cs in command_map.items():
        if len(cs)>1:
            counts['duplicate_command_records'] += len(cs)-1
            if len({digest(c) for c in cs})>1:
                counts['conflicting_command_ids'] += 1
                conflicting_sources.update((c.get('camera_instance'),c.get('source_sequence')) for c in cs)
                continue
        c=cs[0]
        key=(c.get('camera_instance'),c.get('source_sequence'))
        if key not in frame_map: counts['command_without_frame']+=1
        else: by_source[key].append(c)
    rows=[]
    for f in frames:
        key=(f['camera_instance'],f['source_sequence']); cs=by_source.get(key,[])
        row={**f,'match':'unlabeled','command':None,'transmissions':[]}
        if len(frame_map[key])>1: row['match']='duplicate_source_frame'
        elif key in conflicting_sources: row['match']='conflicting_command_id'
        elif len(cs)>1: row['match']='frame_reprocessed_ambiguous'
        elif cs:
            c=cs[0]
            if c['source_stamp_ns']!=f['stamp_ns']: row['match']='stamp_mismatch'
            elif c.get('source_clock_domain')!=f.get('clock_domain'): row['match']='clock_domain_mismatch'
            else:
                row.update(match='direct_source_id' if not f.get('legacy') else 'legacy_exact_unique_stamp',command=c,
                           transmissions=tx_map.get(c['command_id'],[]))
                expected=f"s{c['steering']}l{c['left_pwm']}r{c['right_pwm']}\n"
                moving=[t for t in row['transmissions'] if t.get('payload','').startswith('s')]
                row['executed_request_confirmed']=any(t.get('armed') and t.get('write_success') and t.get('payload')==expected for t in moving)
                row['teacher_transmission_differs']=any(t.get('payload')!=expected for t in moving)
        counts[row['match']]+=1; rows.append(row)
    for cam in {f['camera_instance'] for f in frames}:
        seq=sorted({f['source_sequence'] for f in frames if f['camera_instance']==cam})
        counts['source_sequence_gaps']+=sum(max(0,b-a-1) for a,b in zip(seq,seq[1:]))
    txids=[t.get('transmission_id') for t in transmissions if t.get('transmission_id')]
    counts['duplicate_transmission_records']=len(txids)-len(set(txids))
    counts['transmissions']=len(transmissions)
    counts['transmission_without_command']=sum(t.get('command_id') not in command_map for t in transmissions)
    counts['frames']=len(frames);counts['unique_commands']=len(command_map)
    return rows,dict(counts)


def import_session(source, target, progress=lambda *x:None):
    source=Path(source).expanduser().resolve();target=Path(target)
    if not source.is_dir(): raise ValueError('Session directory not found')
    try:manifest=read_json(source/'manifest.json') if (source/'manifest.json').exists() else {'schema':0,'closed_cleanly':False}
    except ValueError as e:manifest={'schema':0,'closed_cleanly':False,'manifest_parse_error':str(e)}
    run=manifest.get('run_id')
    target.mkdir(parents=True,exist_ok=False)
    store=typestore();frames=[];commands=[];transmissions=[];timing=[];failures=[];camera_status=[]
    databases=sorted(source.rglob('*.db3'))
    if not databases: raise ValueError('No sqlite3 rosbag files (.db3). MCAP must first be converted to sqlite3.')
    total=0
    for db in databases:
        with sqlite3.connect(f'file:{db}?mode=ro',uri=True) as con:
            total+=con.execute('select count(*) from messages').fetchone()[0]
    bag_hashes={}
    for db in databases:
        h=hashlib.sha256()
        with db.open('rb') as f:
            for chunk in iter(lambda:f.read(4*1024*1024),b''):h.update(chunk)
        bag_hashes[str(db.relative_to(source))]=h.hexdigest()
    run=run or 'legacy-'+digest(sorted(bag_hashes.values()))[:24]
    done=0
    for db in databases:
        with sqlite3.connect(f'file:{db}?mode=ro',uri=True) as con:
            topics={i:(n,t) for i,n,t in con.execute('select id,name,type from topics')}
            for mid,tid,received,data in con.execute('select id,topic_id,timestamp,data from messages order by id'):
                name,kind=topics[tid];done+=1
                if done%30==0: progress(done,total)
                if kind not in ('interfaces_pkg/msg/SourceFrame','interfaces_pkg/msg/TeacherCommand','sensor_msgs/msg/Image','std_msgs/msg/String'):continue
                try:
                    m=store.deserialize_cdr(data,kind)
                    if kind in ('interfaces_pkg/msg/SourceFrame','sensor_msgs/msg/Image'):
                        if kind=='sensor_msgs/msg/Image' and ('debug' in name or 'bev' in name):continue
                        wrapped=kind.endswith('SourceFrame');im=m.image if wrapped else m
                        f=dict(camera_instance=m.camera_instance if wrapped else 'legacy',source_sequence=int(m.sequence) if wrapped else stamp(im.header.stamp),
                            stamp_ns=stamp(im.header.stamp),clock_domain=m.clock_domain if wrapped else 'legacy_original_header',
                            header_frame_id=im.header.frame_id,bag_received_ns=int(received),db=str(db.relative_to(source)),message_id=mid,
                            source_image_sha256=hashlib.sha256(im.data.tobytes()).hexdigest(),message_type=kind,width=int(im.width),height=int(im.height),encoding=im.encoding,legacy=not wrapped)
                        if wrapped:
                            f.update(read_started_monotonic_ns=int(m.read_started_monotonic_ns),read_completed_monotonic_ns=int(m.read_completed_monotonic_ns))
                        frames.append(f)
                    elif kind.endswith('TeacherCommand'):commands.append(command_dict(m))
                    elif kind=='std_msgs/msg/String':
                        try: event=json.loads(m.data)
                        except ValueError:continue
                        if event.get('kind')=='transmission':transmissions.append(event)
                        if name=='/e2e/camera_status':camera_status.append(event)
                        if name.endswith('perception_timing'):timing.append(event)
                except Exception as e:failures.append({'db':db.name,'message_id':mid,'error':str(e)})
    if any(not f['legacy'] for f in frames):frames=[f for f in frames if not f['legacy']]
    if any(f['camera_instance'].startswith('DEMO-camera-') for f in frames):
        manifest['demo']=True
        manifest['demo_detection']='Reserved DEMO-camera- prefix in source identities'
    if not commands:
        events,bad=jsonl(source/'events.jsonl')
        timing += [e['data'] for e in events if e.get('kind')=='frame_control']
        unique={}
        for t in timing:
            ns=t.get('frame_stamp_ns')
            if ns is None or t.get('steering') is None:continue
            c=dict(command_id='legacy:'+str(ns),camera_instance='legacy',source_sequence=ns,source_stamp_ns=ns,
                source_clock_domain='legacy_original_header',steering=int(t['steering']),left_pwm=int(t.get('left_pwm') or 0),right_pwm=int(t.get('right_pwm') or 0),
                path_valid=bool(t.get('path_valid')),lane_visible=bool(t.get('lane_currently_visible')),state='TRACK',controller_enabled=True,
                target_lane=t.get('target_lane'),processing_started_monotonic_ns=None,completed_monotonic_ns=None)
            c['command_id']+=':'+digest(c)[:12]
            unique[digest(c)]=c
        commands=list(unique.values())
    rows,stats=match(frames,commands,transmissions)
    rows.sort(key=lambda r:(r['camera_instance'],r['source_sequence']))
    def timing_stats(values):
        return dict(count=len(values),p50_ns=int(np.percentile(values,50)) if values else None,p95_ns=int(np.percentile(values,95)) if values else None)
    periods=[];out_of_order=0
    for cam in {r['camera_instance'] for r in frames}:
        ordered=[r for r in frames if r['camera_instance']==cam]
        out_of_order+=sum(b['source_sequence']<a['source_sequence'] for a,b in zip(ordered,ordered[1:]))
        ordered.sort(key=lambda r:r['source_sequence'])
        periods.extend(b['stamp_ns']-a['stamp_ns'] for a,b in zip(ordered,ordered[1:]) if b['stamp_ns']>a['stamp_ns'])
    process=[c['completed_monotonic_ns']-c['processing_started_monotonic_ns'] for c in commands if c.get('completed_monotonic_ns') is not None and c.get('processing_started_monotonic_ns') is not None]
    writes=[t['write_completed_monotonic_ns']-t['write_started_monotonic_ns'] for t in transmissions if t.get('write_completed_monotonic_ns') is not None]
    _,bad_journal=jsonl(source/'events.jsonl')
    stats.update(camera_period=timing_stats(periods),controller_processing=timing_stats(process),serial_write_duration=timing_stats(writes),
        out_of_order_source_records=out_of_order,invalid_event_lines=bad_journal,camera_status=camera_status[-5:])
    stats.update(decode_failures=failures,closed_cleanly=manifest.get('closed_cleanly',False),demo=manifest.get('demo',False))
    save_json(target/'session.json',dict(schema=SCHEMA,run_id=run,origin_run_id=manifest.get('origin_run_id') or run,
        source_relative=os.path.relpath(source,target),manifest=manifest,stats=stats,rows=rows,
        bag_sha256=bag_hashes,commands=commands,transmissions=transmissions,imported_at_ns=time.time_ns(),training_status='labeled' if commands else 'video_only_no_labels'))
    progress(total,total)
    return read_json(target/'session.json')


def frame_image(session_dir, row):
    session_dir=Path(session_dir);s=read_json(session_dir/'session.json');source=(session_dir/s['source_relative']).resolve()
    db=(source/row['db']).resolve()
    if not db.is_relative_to(source):raise ValueError('Invalid bag path')
    with sqlite3.connect(f'file:{db}?mode=ro',uri=True) as con:
        raw=con.execute('select data from messages where id=?',(row['message_id'],)).fetchone()[0]
    m=typestore().deserialize_cdr(raw,row['message_type']);im=m.image if row['message_type'].endswith('SourceFrame') else m
    if im.encoding.lower() not in ('bgr8','rgb8','8uc3'):raise ValueError('Unsupported image encoding '+im.encoding)
    array=np.frombuffer(im.data,dtype=np.uint8).reshape(int(im.height),int(im.step))[:,:int(im.width)*3].reshape(int(im.height),int(im.width),3).copy()
    return cv2.cvtColor(array,cv2.COLOR_RGB2BGR) if im.encoding.lower()=='rgb8' else array


def review_rows(directory):
    s=read_json(Path(directory)/'session.json');rows=[dict(r,excluded=False,label=(r.get('command') or {}).get('steering'),operator_review='unreviewed') for r in s['rows']]
    events,bad=jsonl(Path(directory)/'review.jsonl');undone={e.get('target') for e in events if e['kind']=='undo'}
    for e in events:
        if e['kind']=='undo' or e['id'] in undone:continue
        for i in range(e['start'],min(len(rows),e['end']+1)):
            if e['kind']=='exclude':rows[i]['excluded']=e['value']
            if e['kind']=='label':rows[i]['label']=e['value']
            if e['kind']=='operator_review':rows[i]['operator_review']=e['value']
            if e['kind']=='alignment_note':rows[i]['alignment_note']=e['value']
            if e['kind']=='alignment_bind':
                c=next(c for c in s['commands'] if c['command_id']==e['value'])
                rows[i].update(command=c,label=c['steering'],match='manually_aligned_legacy',alignment_revision=e['id'],
                    transmissions=[t for t in s['transmissions'] if t.get('command_id')==c['command_id']],
                    executed_request_confirmed=False,teacher_transmission_differs=False)
    return rows,events


def edit_review(directory,event):
    rows,events=review_rows(directory);event=dict(event)
    kind=event.get('kind')
    if not event.get('reason','').strip():raise ValueError('A reason is required')
    if kind not in ('exclude','label','operator_review','alignment_note','alignment_bind','undo'):raise ValueError('Unknown review operation')
    if kind=='undo':
        if not any(e['id']==event.get('target') and e['kind']!='undo' for e in events):raise ValueError('Unknown revision')
    else:
        if not (0<=event['start']<=event['end']<len(rows)):raise ValueError('Invalid inclusive frame range')
        if kind=='alignment_bind':
            s=read_json(Path(directory)/'session.json')
            if any(not r.get('legacy') for r in rows[event['start']:event['end']+1]):raise ValueError('Direct source-frame mapping is immutable; only legacy imports allow explicit manual binding')
            if event['start']!=event['end']:raise ValueError('Manual binding applies to one selected frame')
            if not any(c['command_id']==event['value'] for c in s.get('commands',[])):raise ValueError('Unknown command ID')
        if kind=='label' and (not isinstance(event['value'],(int,float)) or not np.isfinite(event['value']) or abs(event['value'])>7):raise ValueError('Steering must be finite -7..7 command steps')
        if kind=='exclude' and not isinstance(event['value'],bool):raise ValueError('Exclude expects boolean')
        if kind=='operator_review' and event['value'] not in ('accepted','rejected','unreviewed'):raise ValueError('Unknown review status')
    event.update(id=str(uuid.uuid4()),created_ns=time.time_ns())
    journal=Path(directory)/'review.jsonl'
    if journal.exists() and journal.read_bytes() and not journal.read_bytes().endswith(b'\n'):
        with journal.open('a') as f:f.write('\n')
    with journal.open('a') as f:f.write(json.dumps(event)+'\n');f.flush();os.fsync(f.fileno())
    return event


def create_dataset(work,config,progress=lambda *x:None):
    work=Path(work);split=config['split'];selected=[];origins={};journals={};seen_sources={};rejected=collections.Counter()
    if set(split.values())-{'train','validation','test'}:raise ValueError('Unknown split')
    for sid,partition in split.items():
        directory=work/'sessions'/sid;s=read_json(directory/'session.json');origin=s['origin_run_id']
        if s['manifest'].get('mode','track')!='track':raise ValueError('Only general track-driving sessions are in scope')
        if origin in origins and origins[origin]!=partition:raise ValueError('Split leakage: same origin_run_id in multiple partitions')
        origins[origin]=partition;rows,events=review_rows(directory);journals[sid]=events
        if s['stats']['demo'] and not config.get('allow_demo',False):raise ValueError('Synthetic demo sessions require allow_demo=true')
        if not s['stats']['closed_cleanly'] and not config.get('allow_partial',False):raise ValueError('Partial session requires explicit allow_partial=true')
        interval=int(config.get('sample_interval_ns',0));last={}
        for i,r in enumerate(rows):
            c=r.get('command')
            if not c or r['match'] not in ('direct_source_id','legacy_exact_unique_stamp','manually_aligned_legacy'):rejected['unmatched']+=1;continue
            if r['match']=='manually_aligned_legacy' and not config.get('allow_manual_alignment',False):rejected['manual_alignment']+=1;continue
            if r['match'].startswith('legacy') and not config.get('allow_legacy',False):rejected['legacy']+=1;continue
            if r['excluded'] or r['operator_review']=='rejected':rejected['review_excluded']+=1;continue
            if config.get('require_review',True) and r['operator_review']!='accepted':rejected['unreviewed']+=1;continue
            if not(c['path_valid'] and c['lane_visible'] and c['controller_enabled'] and c['state']=='TRACK' and c['left_pwm']>0 and c['right_pwm']>0):rejected['invalid_control']+=1;continue
            if config.get('require_transmission',True) and (not r.get('executed_request_confirmed') or r.get('teacher_transmission_differs')):rejected['transmission']+=1;continue
            if config.get('target_lane') is not None and c['target_lane']!=config['target_lane']:continue
            if config.get('fixed_pwm') is not None and (c['left_pwm']!=config['fixed_pwm'] or c['right_pwm']!=config['fixed_pwm']):rejected['pwm_filter']+=1;continue
            identity=(r['camera_instance'],r['source_sequence'])
            if identity in seen_sources:
                if seen_sources[identity]!=partition:raise ValueError('Split leakage: same source identity appears in multiple partitions')
                rejected['duplicate_origin_sample']+=1;continue
            cam=r['camera_instance'];ns=r['stamp_ns']
            if cam in last and ns-last[cam]<interval:continue
            last[cam]=ns;seen_sources[identity]=partition
            selected.append(dict(session=sid,origin_run_id=origin,source_sequence=r['source_sequence'],camera_instance=cam,
                frame_index=i,stamp_ns=ns,split=partition,label_steps=float(r['label']),label=float(r['label'])/7,
                teacher_steps=c['steering'],source_image_sha256=r.get('source_image_sha256'),match=r['match']))
    if config.get('balance_max_per_bin'):
        cap=int(config['balance_max_per_bin']);rng=random.Random(config.get('seed',42));rng.shuffle(selected);hist=collections.Counter();kept=[]
        for x in selected:
            key=(x['split'],int(round(x['label_steps'])))
            if x['split']!='train' or hist[key]<cap:kept.append(x);hist[key]+=1
        selected=kept
    if not all(any(x['split']==s for x in selected) for s in ('train','validation','test')):raise ValueError('Each split needs eligible samples from separate runs')
    content=dict(schema=SCHEMA,config=config,review_snapshots=journals,samples=selected)
    version=digest(content);dest=work/'datasets'/version[:16]
    if dest.exists():
        if not (dest/'dataset.json').exists():raise ValueError('Incomplete dataset directory; choose a new configuration or inspect partial output')
        return read_json(dest/'dataset.json')
    dest.mkdir(parents=True);(dest/'images').mkdir()
    for i,x in enumerate(selected):
        sd=work/'sessions'/x['session'];row=read_json(sd/'session.json')['rows'][x['frame_index']]
        image=frame_image(sd,row);name=f'images/{i:08d}.png'
        if not cv2.imwrite(str(dest/name),image):raise IOError('Image write failed')
        x['image']=name;x['image_sha256']=hashlib.sha256((dest/name).read_bytes()).hexdigest();progress(i+1,len(selected))
    content.update(version=version,dataset_id=dest.name,created_ns=time.time_ns(),stats=dict(
        counts=dict(collections.Counter(x['split'] for x in selected)),steering_histogram=dict(collections.Counter(str(x['label_steps']) for x in selected)),rejected=dict(rejected)),
        output=dict(unit='normalized_command_step',scale_steps=7,range=[-1,1],negative='left',wheel_angle_measured=False))
    content['manifest_sha256']=digest(content)
    save_json(dest/'dataset.json',content);return content
