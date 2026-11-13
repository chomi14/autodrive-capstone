import copy
import json
from pathlib import Path
import shutil
import subprocess
import sys

import pytest
from e2e.core import match,import_session,create_dataset,read_json,edit_review,review_rows,frame_image,save_json
from e2e.synthetic import make_demo


def sample():
    f=dict(camera_instance='cam',source_sequence=1,stamp_ns=1790000000123456789,clock_domain='ros_host_system')
    c=dict(command_id='c1',camera_instance='cam',source_sequence=1,source_stamp_ns=f['stamp_ns'],source_clock_domain='ros_host_system',steering=3,left_pwm=80,right_pwm=80)
    t=dict(command_id='c1',transmission_id='t1',payload='s3l80r80\n',write_success=True,armed=True)
    return f,c,t


def test_exact_delayed_reordered_retransmission():
    f,c,t=sample();g={**f,'source_sequence':2,'stamp_ns':f['stamp_ns']+1};d={**c,'command_id':'c2','source_sequence':2,'source_stamp_ns':g['stamp_ns']}
    rows,s=match([g,f],[c,d],[t,{**t,'transmission_id':'t2'}])
    assert s['direct_source_id']==2 and s['transmissions']==2 and len(rows)==2
    assert rows[1]['command']['command_id']=='c1' and rows[1]['executed_request_confirmed']

@pytest.mark.parametrize('change,reason',[(dict(source_stamp_ns=1),'stamp_mismatch'),(dict(source_clock_domain='replay'),'clock_domain_mismatch')])
def test_no_fuzzy_match(change,reason):
    f,c,t=sample();rows,s=match([f],[{**c,**change}],[t]);assert rows[0]['match']==reason


def test_gaps_reprocess_duplicate_conflicts():
    f,c,t=sample();rows,s=match([f,{**f,'source_sequence':4}],[c,{**c,'command_id':'c2'}],[t])
    assert rows[0]['match']=='frame_reprocessed_ambiguous' and s['source_sequence_gaps']==2
    assert match([f,f],[c],[])[1]['duplicate_source_frame']==2
    assert match([f],[c,{**c,'steering':7}],[])[1]['conflicting_command_ids']==1
    assert match([f],[c,{**c,'steering':7}],[])[0][0]['match']=='conflicting_command_id'
    assert match([f],[c,c],[])[1]['duplicate_command_records']==1


def test_transmission_override_and_unmatched():
    f,c,t=sample();rows,s=match([f],[c],[{**t,'payload':'s3l0r0\n'}]);assert rows[0]['teacher_transmission_differs'];assert not rows[0]['executed_request_confirmed']
    assert match([], [c],[t])[1]['command_without_frame']==1

@pytest.fixture
def sessions(tmp_path):
    source=tmp_path/'source';paths=make_demo(source,9);work=tmp_path/'work'
    for i,p in enumerate(paths):
        sd=work/'sessions'/f'r{i}';import_session(p,sd)
        edit_review(sd,dict(kind='operator_review',start=0,end=8,value='accepted',reason='Synthetic test only'))
    return source,work


def config():return dict(split={'r0':'train','r1':'validation','r2':'test'},allow_demo=True,require_review=True,require_transmission=True)


def test_review_undo_frozen_versions_portability(sessions,tmp_path):
    source,work=sessions;sd=work/'sessions/r0'
    e=edit_review(sd,dict(kind='label',start=0,end=1,value=-6.5,reason='test'))
    d=create_dataset(work,config());assert len(d['samples'])==27
    assert d['stats']['counts']==dict(train=9,validation=9,test=9)
    before=(work/'datasets'/d['dataset_id']/'dataset.json').read_bytes()
    edit_review(sd,dict(kind='undo',target=e['id'],reason='undo test'));assert review_rows(sd)[0][0]['label']!=-6.5
    d2=create_dataset(work,config());assert d2['version']!=d['version']
    assert before==(work/'datasets'/d['dataset_id']/'dataset.json').read_bytes()
    copied=tmp_path/'portable';shutil.copytree(work,copied/'work');shutil.copytree(source,copied/'source')
    assert frame_image(copied/'work/sessions/r0',read_json(copied/'work/sessions/r0/session.json')['rows'][0]).shape==(120,200,3)


def test_leakage_and_duplicate_origin(sessions):
    source,work=sessions;shutil.copytree(work/'sessions/r0',work/'sessions/derived')
    cfg=config();cfg['split']['derived']='test'
    with pytest.raises(ValueError,match='leakage'):create_dataset(work,cfg)
    cfg['split']['derived']='train';d=create_dataset(work,cfg);assert len(d['samples'])==27
    assert d['stats']['rejected']['duplicate_origin_sample']==9


def test_partial_and_truncated_journal(sessions):
    _,work=sessions;sd=work/'sessions/r0';s=read_json(sd/'session.json');s['stats']['closed_cleanly']=False;save_json(sd/'session.json',s)
    with (sd/'review.jsonl').open('a') as f:f.write('{"truncated":')
    with pytest.raises(ValueError,match='Partial'):create_dataset(work,config())
    cfg=config();cfg['allow_partial']=True;assert len(create_dataset(work,cfg)['samples'])==27


def test_invalid_label_and_no_review(sessions):
    _,work=sessions;sd=work/'sessions/r0'
    with pytest.raises(ValueError):edit_review(sd,dict(kind='label',start=0,end=0,value=8,reason='test'))
    with pytest.raises(ValueError):edit_review(sd,dict(kind='label',start=0,end=0,value=1,reason=''))
    edit_review(sd,dict(kind='exclude',start=0,end=8,value=True,reason='all excluded'))
    with pytest.raises(ValueError,match='Each split'):create_dataset(work,config())


def test_partial_bag_without_metadata_and_video_only(sessions,tmp_path):
    source,work=sessions
    original=source/'demo_run_1';partial=tmp_path/'partial';shutil.copytree(original,partial)
    for f in partial.rglob('metadata.yaml'):f.rename(f.with_suffix('.yaml.preserved'))
    m=read_json(partial/'manifest.json');m['closed_cleanly']=False;save_json(partial/'manifest.json',m)
    recovered=import_session(partial,tmp_path/'recovered');assert recovered['stats']['direct_source_id']==9 and not recovered['stats']['closed_cleanly']
    # Video-only bag: preserve the source fixture, write a new bag with only source images.
    from rosbags.rosbag2 import Reader,Writer
    from e2e.core import typestore
    video=tmp_path/'video';video.mkdir();store=typestore()
    with Reader(original/'input_bag') as reader,Writer(video/'input_bag',version=8) as writer:
        conn=writer.add_connection('/e2e/source_frame','interfaces_pkg/msg/SourceFrame',typestore=store)
        for c,ns,raw in reader.messages():
            if c.msgtype==conn.msgtype:writer.write(conn,ns,raw)
    imported=import_session(video,tmp_path/'video_index');assert imported['training_status']=='video_only_no_labels' and imported['stats']['unlabeled']==9


def test_legacy_exact_matching_and_explicit_alignment(sessions,tmp_path):
    source,work=sessions
    from rosbags.rosbag2 import Reader,Writer
    from e2e.core import typestore
    store=typestore();legacy=tmp_path/'legacy';legacy.mkdir();events=[]
    save_json(legacy/'manifest.json',dict(schema=1,mode='track'))
    with Reader(source/'demo_run_1/input_bag') as reader,Writer(legacy/'input_bag',version=8) as writer:
        conn=writer.add_connection('/track/image_raw','sensor_msgs/msg/Image',typestore=store)
        for c,ns,raw in reader.messages():
            if c.msgtype=='interfaces_pkg/msg/SourceFrame':
                m=store.deserialize_cdr(raw,c.msgtype);writer.write(conn,ns,store.serialize_cdr(m.image,conn.msgtype))
                events.append(dict(kind='frame_control',data=dict(frame_stamp_ns=ns,steering=2,left_pwm=80,right_pwm=80,path_valid=True,lane_currently_visible=True)))
    (legacy/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))
    target=tmp_path/'legacy_index';s=import_session(legacy,target);assert s['stats']['legacy_exact_unique_stamp']==9
    edit_review(target,dict(kind='alignment_bind',start=0,end=0,value=s['commands'][1]['command_id'],reason='Explicit legacy alignment correction test'))
    r,_=review_rows(target);assert r[0]['match']=='manually_aligned_legacy' and r[0]['stamp_ns']==s['rows'][0]['stamp_ns']
    direct=work/'sessions/r0';d=read_json(direct/'session.json')
    with pytest.raises(ValueError,match='immutable'):edit_review(direct,dict(kind='alignment_bind',start=0,end=0,value=d['commands'][1]['command_id'],reason='Not permitted'))
    moved=tmp_path/'legacy_moved';shutil.copytree(legacy,moved);s2=import_session(moved,tmp_path/'legacy_index2');assert s2['run_id']==s['run_id']
