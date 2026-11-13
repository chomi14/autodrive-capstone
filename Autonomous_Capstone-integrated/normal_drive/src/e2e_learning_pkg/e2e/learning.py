"""PilotNet regression. The same preprocess() is used by training, review and ROS shadow."""
import json
import math
import os
from pathlib import Path
import random
import signal
import time

import cv2
import numpy as np
import torch
from torch import nn
from torch.utils.data import Dataset, DataLoader

from .core import read_json, save_json, digest

DEFAULTS=dict(architecture='pilotnet',mode='new',checkpoint='',epochs=20,batch_size=32,learning_rate=0.001,
    optimizer='adamw',loss='huber',scheduler='cosine',weight_decay=0.0001,seed=42,early_stopping=5,
    device='cpu',threads=2,preprocess=dict(crop=[0.35,1.0,0.0,1.0],width=200,height=66,color='yuv',normalization='minus_one_one'),
    augmentation=dict(horizontal_flip=False,brightness=0.10))
FORMAT='normal-drive-pilotnet-v1'


def preprocess(bgr,config):
    top,bottom,left,right=config['crop'];h,w=bgr.shape[:2]
    if not (0<=top<bottom<=1 and 0<=left<right<=1):raise ValueError('Crop uses [top,bottom,left,right] fractions in 0..1')
    image=bgr[int(top*h):int(bottom*h),int(left*w):int(right*w)]
    if image.size==0:raise ValueError('Empty input crop')
    image=cv2.resize(image,(int(config['width']),int(config['height'])),interpolation=cv2.INTER_AREA)
    if config['color']=='yuv':image=cv2.cvtColor(image,cv2.COLOR_BGR2YUV)
    elif config['color']=='rgb':image=cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
    else:raise ValueError('Color must be rgb or yuv')
    if config['normalization']!='minus_one_one':raise ValueError('Unsupported normalization')
    return np.ascontiguousarray(image.transpose(2,0,1),dtype=np.float32)/127.5-1.


class PilotNet(nn.Module):
    def __init__(self,width=200,height=66):
        super().__init__()
        self.conv=nn.Sequential(nn.Conv2d(3,24,5,2),nn.ELU(),nn.Conv2d(24,36,5,2),nn.ELU(),nn.Conv2d(36,48,5,2),nn.ELU(),
            nn.Conv2d(48,64,3),nn.ELU(),nn.Conv2d(64,64,3),nn.ELU(),nn.Flatten())
        with torch.no_grad():size=self.conv(torch.zeros(1,3,height,width)).shape[1]
        self.head=nn.Sequential(nn.Linear(size,100),nn.ELU(),nn.Linear(100,50),nn.ELU(),nn.Linear(50,10),nn.ELU(),nn.Linear(10,1),nn.Tanh())
    def forward(self,x):return self.head(self.conv(x)).squeeze(-1)


def load_checkpoint(path,device='cpu'):
    try: checkpoint=torch.load(path,map_location=device,weights_only=True)
    except Exception as e:raise ValueError('Invalid E2E checkpoint (YOLO .pt is not accepted): '+str(e)) from e
    if not isinstance(checkpoint,dict) or checkpoint.get('format')!=FORMAT:raise ValueError('Not a normal-drive PilotNet checkpoint; YOLO weights are incompatible')
    if checkpoint.get('output')!={'unit':'normalized_command_step','scale_steps':7,'range':[-1,1],'negative':'left'}:raise ValueError('Unsupported output contract')
    p=checkpoint['preprocess'];model=PilotNet(p['width'],p['height']).to(device);model.load_state_dict(checkpoint['model']);model.eval()
    return model,checkpoint


class Samples(Dataset):
    def __init__(self,directory,rows,config,train=False):self.directory=Path(directory);self.rows=rows;self.config=config;self.train=train
    def __len__(self):return len(self.rows)
    def __getitem__(self,i):
        r=self.rows[i];image=cv2.imread(str(self.directory/r['image']))
        if image is None:raise ValueError('Missing dataset image '+r['image'])
        y=float(r['label']);aug=self.config['augmentation']
        if self.train:
            if aug.get('horizontal_flip') and torch.rand(()).item()<.5:image=cv2.flip(image,1);y=-y
            brightness=float(aug.get('brightness',0))
            if brightness:image=np.clip(image.astype(np.float32)*(1+(torch.rand(()).item()*2-1)*brightness),0,255).astype(np.uint8)
        return torch.from_numpy(preprocess(image,self.config['preprocess'])),torch.tensor(y,dtype=torch.float32)


def validate_dataset(directory):
    directory=Path(directory);d=read_json(directory/'dataset.json');integrity=d.get('manifest_sha256')
    if not integrity or digest({k:v for k,v in d.items() if k!='manifest_sha256'})!=integrity:raise ValueError('Frozen dataset manifest integrity check failed')
    origins={};identities=set()
    for r in d['samples']:
        origin=r['origin_run_id']
        if origin in origins and origins[origin]!=r['split']:raise ValueError('Origin leakage in dataset')
        origins[origin]=r['split'];identity=(origin,r['camera_instance'],r['source_sequence'])
        if identity in identities:raise ValueError('Duplicate source sample')
        identities.add(identity)
        if not np.isfinite(r['label']) or abs(r['label'])>1:raise ValueError('Label out of bounds')
        import hashlib
        if hashlib.sha256((directory/r['image']).read_bytes()).hexdigest()!=r['image_sha256']:raise ValueError('Frozen image hash changed')
    return d


def train(directory,output,settings):
    os.environ.setdefault('CUBLAS_WORKSPACE_CONFIG',':4096:8')
    cfg=json.loads(json.dumps(DEFAULTS));cfg.update(settings)
    if cfg['architecture']!='pilotnet':raise ValueError('Only pilotnet is supported')
    if cfg['mode'] not in ('new','finetune','resume'):raise ValueError('Unknown training mode')
    if int(cfg['epochs'])<1 or int(cfg['batch_size'])<1 or not 0<float(cfg['learning_rate'])<1:raise ValueError('Invalid training parameters')
    torch.set_num_threads(int(cfg['threads']));torch.manual_seed(cfg['seed']);random.seed(cfg['seed']);np.random.seed(cfg['seed']);torch.use_deterministic_algorithms(True)
    device=cfg['device']
    if str(device).startswith('cuda') and not torch.cuda.is_available():raise ValueError('CUDA device unavailable; select cpu')
    directory=Path(directory);output=Path(output);output.mkdir(parents=True,exist_ok=True);d=validate_dataset(directory)
    train_rows=[r for r in d['samples'] if r['split']=='train'];val_rows=[r for r in d['samples'] if r['split']=='validation']
    if not train_rows or not val_rows:raise ValueError('Training and validation splits are required')
    checkpoint=None;start=0;history=[];best=float('inf');bad=0;best_state=None;best_epoch=None
    if cfg['mode']!='new':
        model,checkpoint=load_checkpoint(cfg['checkpoint'],device)
        if cfg['preprocess']!=checkpoint['preprocess']:raise ValueError('Checkpoint preprocessing differs')
    else:
        if cfg['checkpoint']:raise ValueError('New training must not specify a checkpoint')
        p=cfg['preprocess'];model=PilotNet(p['width'],p['height']).to(device)
    opts={'adamw':torch.optim.AdamW,'adam':torch.optim.Adam,'sgd':torch.optim.SGD}
    if cfg['optimizer'] not in opts:raise ValueError('Unknown optimizer')
    optimizer=opts[cfg['optimizer']](model.parameters(),lr=cfg['learning_rate'],weight_decay=cfg['weight_decay'])
    losses={'mse':nn.MSELoss,'huber':nn.SmoothL1Loss,'mae':nn.L1Loss}
    if cfg['loss'] not in losses:raise ValueError('Unknown loss')
    criterion=losses[cfg['loss']]()
    if cfg['scheduler']=='cosine':scheduler=torch.optim.lr_scheduler.CosineAnnealingLR(optimizer,T_max=cfg['epochs'])
    elif cfg['scheduler']=='plateau':scheduler=torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer,patience=2)
    elif cfg['scheduler']=='none':scheduler=None
    else:raise ValueError('Unknown scheduler')
    if cfg['mode']=='resume':
        if not checkpoint.get('resume_available',True):raise ValueError('This recovered best checkpoint is for inference/finetuning; resume from last.pt')
        if checkpoint['dataset_version']!=d['version']:raise ValueError('Resume requires identical frozen dataset')
        old=checkpoint['config']
        for k in ('optimizer','loss','scheduler','weight_decay','seed','augmentation','batch_size','learning_rate','epochs','early_stopping'):
            if cfg[k]!=old[k]:raise ValueError('Resume requires unchanged '+k+'; use finetune for changed training settings')
        optimizer.load_state_dict(checkpoint['optimizer'])
        if scheduler:scheduler.load_state_dict(checkpoint['scheduler'])
        best_state=checkpoint.get('best_model');best_epoch=checkpoint.get('best_epoch')
        previous_best=Path(cfg['checkpoint']).parent/'best.pt'
        if previous_best.exists() and previous_best.resolve()!=(output/'best.pt').resolve():
            import shutil
            shutil.copy2(previous_best,output/'best.pt')
            if best_state is None:
                prior=torch.load(previous_best,map_location='cpu',weights_only=True)
                best_state=prior['model'];best_epoch=prior['epoch']
        elif best_state is not None and not (output/'best.pt').exists():
            recovered={**checkpoint,'model':best_state,'epoch':best_epoch,'optimizer':None,'scheduler':None,'resume_available':False}
            torch.save(recovered,output/'best.pt')
        elif best_state is None:
            raise ValueError('Legacy resume checkpoint requires sibling best.pt; new checkpoints embed best weights')
        start=checkpoint['epoch']+1;history=checkpoint['history'];best=checkpoint['best_loss'];bad=checkpoint['bad_epochs']
        torch.set_rng_state(checkpoint['torch_rng'].cpu());random.setstate(checkpoint['python_rng'])
        if torch.cuda.is_available() and checkpoint.get('cuda_rng'):torch.cuda.set_rng_state_all([state.cpu() for state in checkpoint['cuda_rng']])
    stopped=[False]
    def stop(*_):stopped[0]=True
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    val_loader=DataLoader(Samples(directory,val_rows,cfg),batch_size=cfg['batch_size'],shuffle=False,num_workers=0)
    save_json(output/'config.json',{**cfg,'dataset_version':d['version'],'dataset_directory':os.path.relpath(directory,output)})
    for epoch in range(start,cfg['epochs']):
        began=time.monotonic();model.train();total=0.;n=0
        generator=torch.Generator().manual_seed(cfg['seed']+epoch)
        loader=DataLoader(Samples(directory,train_rows,cfg,True),batch_size=cfg['batch_size'],shuffle=True,generator=generator,num_workers=0)
        for x,y in loader:
            x,y=x.to(device),y.to(device);optimizer.zero_grad();loss=criterion(model(x),y);loss.backward();optimizer.step();total+=float(loss.item())*len(x);n+=len(x)
        model.eval();vtotal=0.
        with torch.no_grad():
            for x,y in val_loader:vtotal+=float(criterion(model(x.to(device)),y.to(device)).item())*len(x)
        val=vtotal/len(val_rows)
        if scheduler:
            if cfg['scheduler']=='plateau':scheduler.step(val)
            else:scheduler.step()
        improved=val<best
        if improved:
            best=val;bad=0;best_epoch=epoch
            best_state={k:v.detach().cpu().clone() for k,v in model.state_dict().items()}
        else:bad+=1
        history.append(dict(epoch=epoch+1,train_loss=total/n,validation_loss=val,gap=val-total/n,seconds=time.monotonic()-began))
        record=dict(format=FORMAT,architecture='pilotnet',model=model.state_dict(),preprocess=cfg['preprocess'],
            output={'unit':'normalized_command_step','scale_steps':7,'range':[-1,1],'negative':'left'},dataset_version=d['version'],config=cfg,
            optimizer=optimizer.state_dict(),scheduler=scheduler.state_dict() if scheduler else None,
            epoch=epoch,best_loss=best,best_model=best_state,best_epoch=best_epoch,resume_available=True,bad_epochs=bad,history=history,torch_rng=torch.get_rng_state(),python_rng=random.getstate(),
            cuda_rng=torch.cuda.get_rng_state_all() if torch.cuda.is_available() else [],torch_version=str(torch.__version__))
        torch.save(record,output/'last.tmp');os.replace(output/'last.tmp',output/'last.pt')
        if improved:
            torch.save(record,output/'best.tmp');os.replace(output/'best.tmp',output/'best.pt')
        save_json(output/'metrics.json',dict(history=history,epoch=epoch+1,total_epochs=cfg['epochs'],state='stopping' if stopped[0] else 'running'))
        print(json.dumps(history[-1]),flush=True)
        if stopped[0] or (cfg['early_stopping']>0 and bad>=cfg['early_stopping']):break
    if history:
        save_json(output/'metrics.json',dict(history=history,epoch=history[-1]['epoch'],total_epochs=cfg['epochs'],state='stopped' if stopped[0] else 'completed',
            note='Offline teacher imitation only; no closed-loop driving performance measured'))
    else:raise ValueError('Checkpoint has already completed requested epochs')
    return history


def evaluate(directory,checkpoints,output,device='cpu'):
    torch.set_num_threads(2);d=validate_dataset(directory);rows=[r for r in d['samples'] if r['split']=='test']
    if not rows:raise ValueError('Fixed test split is empty')
    results=[]
    for path in checkpoints:
        model,c=load_checkpoint(path,device)
        if c['dataset_version']!=d['version']:raise ValueError('Evaluation requires the same frozen dataset/test split')
        predictions=[];times=[]
        def sync():
            if str(device).startswith('cuda'):torch.cuda.synchronize()
        dummy=torch.from_numpy(preprocess(cv2.imread(str(Path(directory)/rows[0]['image'])),c['preprocess'])).unsqueeze(0).to(device)
        with torch.inference_mode():
            for _ in range(5):model(dummy)
            sync()
            for r in rows:
                bgr=cv2.imread(str(Path(directory)/r['image']));sync();began=time.perf_counter_ns()
                x=torch.from_numpy(preprocess(bgr,c['preprocess'])).unsqueeze(0).to(device);pred=float(model(x).item());sync()
                elapsed=time.perf_counter_ns()-began;times.append(elapsed/1e6)
                predictions.append(dict(session=r['session'],frame_index=r['frame_index'],stamp_ns=r['stamp_ns'],truth_steps=r['label_steps'],prediction_steps=pred*7,error_steps=abs(pred*7-r['label_steps']),latency_ns=elapsed))
        errors=np.array([p['prediction_steps']-p['truth_steps'] for p in predictions]);buckets={}
        for name,fn in [('left',lambda y:y<-2),('center',lambda y:abs(y)<=2),('right',lambda y:y>2)]:
            es=[p['error_steps'] for p in predictions if fn(p['truth_steps'])];buckets[name]={'count':len(es),'mae_steps':float(np.mean(es)) if es else None}
        deltas=[abs(b['prediction_steps']-a['prediction_steps']) for a,b in zip(predictions,predictions[1:]) if a['session']==b['session']]
        results.append(dict(checkpoint=str(path),mae_steps=float(np.mean(np.abs(errors))),rmse_steps=float(np.sqrt(np.mean(errors**2))),
            error_by_steering=buckets,mean_prediction_delta_steps=float(np.mean(deltas)) if deltas else None,
            latency_ms=dict(p50=float(np.percentile(times,50)),p95=float(np.percentile(times,95)),device=device,threads=2,batch_size=1,
                            warmup=5,scope='preprocess + device copy + forward, excludes disk decode',torch=str(torch.__version__)),predictions=predictions,
            worst_frames=sorted(predictions,key=lambda p:p['error_steps'],reverse=True)[:20]))
    report=dict(dataset_version=d['version'],split='fixed_test_sessions',evaluation='offline_open_loop_teacher_imitation',closed_loop_measured=False,models=results)
    save_json(output,report);return report


def import_model(source,destination):
    import hashlib,shutil
    source=Path(source).expanduser().resolve()
    model,c=load_checkpoint(source,'cpu')
    model_id=hashlib.sha256(source.read_bytes()).hexdigest()[:16]
    target=Path(destination)/model_id;target.mkdir(parents=True,exist_ok=True)
    shutil.copy2(source,target/'checkpoint.pt')
    sibling=source.parent/'best.pt'
    if sibling.exists():
        _,prior=load_checkpoint(sibling,'cpu')
        if prior['dataset_version']==c['dataset_version']:shutil.copy2(sibling,target/'best.pt')
    metadata=dict(id=model_id,format=c['format'],architecture=c['architecture'],preprocess=c['preprocess'],output=c['output'],dataset_version=c['dataset_version'],config=c['config'])
    save_json(target/'model.json',metadata)
    return metadata
