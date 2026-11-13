"""Local-only HTTP API. Long work runs in child processes; no ROS/vehicle API."""
import json
import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import threading
import time
import traceback
from http.server import ThreadingHTTPServer,BaseHTTPRequestHandler
from urllib.parse import urlparse,parse_qs

import cv2
from .core import read_json,save_json,import_session,review_rows,edit_review,frame_image


def serve(work,port):
    work=Path(work).resolve();work.mkdir(parents=True,exist_ok=True)
    for sub in ('sessions','jobs','datasets','training','evaluations','models'):(work/sub).mkdir(exist_ok=True)
    token=secrets.token_hex(24);processes={};lock=threading.Lock()
    static=Path(__file__).parent/'static'
    def local(folder,key):
        path=(work/folder/key).resolve()
        if not path.is_relative_to(work/folder):raise ValueError('Invalid identifier')
        return path
    def job(kind,args,config=None):
        jid=secrets.token_hex(8);out=local('jobs',jid);out.mkdir();config=config or {}
        save_json(out/'request.json',config)
        args=[str(out/'request.json') if a=='$CONFIG' else str(a) for a in args]
        if kind in ('import','dataset'):args=['--progress-file',str(out/'progress.json')]+args
        log=(out/'process.log').open('w');proc=subprocess.Popen([sys.executable,'-m','e2e.cli',*args],stdout=log,stderr=subprocess.STDOUT,start_new_session=True);log.close()
        record=dict(id=jid,kind=kind,pid=proc.pid,started_ns=time.time_ns(),state='running',config=config)
        save_json(out/'job.json',record)
        with lock:processes[jid]=proc
        return record
    def jobs():
        result=[]
        for f in (work/'jobs').glob('*/job.json'):
            j=read_json(f);proc=processes.get(j['id'])
            if proc and proc.poll() is not None and j['state'] in ('running','stopping'):
                j.update(state='completed' if proc.returncode==0 else 'failed',returncode=proc.returncode);save_json(f,j)
            elif not proc and j['state'] in ('running','stopping'):j['state']='orphaned_inspect_log'
            j['progress']=read_json(f.parent/'progress.json') if (f.parent/'progress.json').exists() else None
            j['log']=(f.parent/'process.log').read_text(errors='replace')[-12000:]
            result.append(j)
        return sorted(result,key=lambda j:j['started_ns'],reverse=True)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def send(self,data,kind='application/json',status=200):
            if kind=='application/json':
                def integers(obj):
                    if isinstance(obj,dict):return {k:(str(v) if isinstance(v,int) and (k.endswith('_ns') or k=='source_sequence') else integers(v)) for k,v in obj.items()}
                    if isinstance(obj,list):return [integers(x) for x in obj]
                    return obj
                data=json.dumps(integers(data),allow_nan=False).encode()
            elif isinstance(data,str):data=data.encode()
            self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data)
        def do_GET(self):
            try:
                u=urlparse(self.path);q=parse_qs(u.query);path=u.path
                if path=='/api/state':
                    sessions=[]
                    for f in (work/'sessions').glob('*/session.json'):
                        s=read_json(f);sessions.append(dict(id=f.parent.name,run_id=s['run_id'],manifest=s['manifest'],stats=s['stats'],training_status=s['training_status'],origin_run_id=s['origin_run_id']))
                    datasets=[dict(id=f.parent.name,version=read_json(f)['version'],stats=read_json(f)['stats']) for f in (work/'datasets').glob('*/dataset.json')]
                    training=[]
                    for f in (work/'training').glob('*/config.json'):
                        training.append(dict(id=f.parent.name,config=read_json(f),metrics=read_json(f.parent/'metrics.json') if (f.parent/'metrics.json').exists() else None,checkpoints=[str(x.relative_to(work)) for x in f.parent.glob('*.pt')]))
                    evaluations=[dict(id=f.stem,report=read_json(f)) for f in (work/'evaluations').glob('*.json')]
                    return self.send(dict(token=token,sessions=sessions,datasets=datasets,jobs=jobs(),training=training,evaluations=evaluations,models=[dict(id=f.parent.name,checkpoint=str((f.parent/'checkpoint.pt').relative_to(work)),metadata=read_json(f)) for f in (work/'models').glob('*/model.json')],work=str(work)))
                if path=='/api/session':
                    directory=local('sessions',q['id'][0]);s=read_json(directory/'session.json');rows,events=review_rows(directory)
                    # Integer ns cross the JS boundary as decimal strings (avoid >2**53 precision loss).
                    def strings(obj):
                        if isinstance(obj,dict):return {k:(str(v) if (k.endswith('_ns') or k=='source_sequence') and isinstance(v,int) else strings(v)) for k,v in obj.items()}
                        if isinstance(obj,list):return [strings(x) for x in obj]
                        return obj
                    return self.send(strings(dict(session=s,rows=rows,revisions=events)))
                if path=='/api/image':
                    directory=local('sessions',q['id'][0]);s=read_json(directory/'session.json');i=int(q['frame'][0]);image=frame_image(directory,s['rows'][i])
                    if 'crop' in q:
                        from .learning import preprocess,DEFAULTS
                        pre=json.loads(q['config'][0]) if 'config' in q else DEFAULTS['preprocess']
                        a=preprocess(image,pre);image=np_preview(a,pre)
                    ok,data=cv2.imencode('.jpg',image)
                    if not ok:raise ValueError('Preview encode failed')
                    return self.send(data.tobytes(),'image/jpeg')
                file=static/('index.html' if path=='/' else path.lstrip('/'))
                if not file.resolve().is_relative_to(static) or not file.is_file():return self.send({'error':'Not found'},status=404)
                kind='text/html; charset=utf-8' if file.suffix=='.html' else 'text/javascript' if file.suffix=='.js' else 'text/css'
                return self.send(file.read_bytes(),kind)
            except Exception as e:self.send({'error':str(e)},status=400)
        def do_POST(self):
            try:
                if self.headers.get('X-E2E-Key')!=token:return self.send({'error':'Local request token required'},status=403)
                length=int(self.headers.get('Content-Length',0))
                if length>1_000_000:raise ValueError('Use local folder import for large bags')
                b=json.loads(self.rfile.read(length));path=urlparse(self.path).path
                if path=='/api/import':
                    sid=secrets.token_hex(8);return self.send(job('import',['import',b['path'],local('sessions',sid)],b))
                if path=='/api/review':return self.send(edit_review(local('sessions',b['session']),b['event']))
                if path=='/api/dataset':return self.send(job('dataset',['dataset','--work',work,'--config','$CONFIG'],b))
                if path=='/api/model-import':return self.send(job('model-import',['model-import',b['path'],work/'models'],b))
                if path=='/api/train':
                    tid=secrets.token_hex(8);config=b['config']
                    if config.get('checkpoint'):config['checkpoint']=str(local('',config['checkpoint']))
                    return self.send(job('train',['train','--dataset',local('datasets',b['dataset']),'--output',local('training',tid),'--config','$CONFIG'],config))
                if path=='/api/stop':
                    proc=processes.get(b['job'])
                    if not proc or proc.poll() is not None:raise ValueError('Job is not running')
                    proc.send_signal(signal.SIGTERM);return self.send({'state':'stopping','note':'Training saves checkpoint at the next epoch boundary'})
                if path=='/api/evaluate':
                    eid=secrets.token_hex(8)
                    return self.send(job('evaluate',['evaluate','--dataset',local('datasets',b['dataset']),'--checkpoints',*[local('',c) for c in b['checkpoints']],'--output',local('evaluations',eid+'.json'),'--device',b.get('device','cpu')],b))
                raise ValueError('Unknown endpoint')
            except Exception as e:self.send({'error':str(e)},status=400)
    server=ThreadingHTTPServer(('127.0.0.1',port),Handler)
    print(f'Local workbench http://127.0.0.1:{port} (no vehicle connection)',flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close()


def np_preview(array,config):
    import numpy as np
    image=np.clip((array.transpose(1,2,0)+1)*127.5,0,255).astype(np.uint8)
    return cv2.cvtColor(image,cv2.COLOR_YUV2BGR if config['color']=='yuv' else cv2.COLOR_RGB2BGR)
