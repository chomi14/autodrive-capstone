import json
from pathlib import Path
import subprocess
import sys
import torch
import pytest
from e2e.learning import load_checkpoint,preprocess,DEFAULTS
from e2e.core import create_dataset,edit_review,import_session,save_json,read_json
from e2e.synthetic import make_demo


def test_checkpoint_rejection_and_shared_preprocess(tmp_path):
    torch.save({'model':'YOLO dummy'},tmp_path/'yolo.pt')
    with pytest.raises(ValueError,match='YOLO'):load_checkpoint(tmp_path/'yolo.pt')
    import numpy as np
    image=np.zeros((120,200,3),np.uint8);x=preprocess(image,DEFAULTS['preprocess']);assert x.shape==(3,66,200) and x.dtype==np.float32


def test_train_finetune_evaluate_and_frozen_integrity(tmp_path):
    paths=make_demo(tmp_path/'source',6);work=tmp_path/'work'
    for i,p in enumerate(paths):
        sd=work/'sessions'/str(i);import_session(p,sd);edit_review(sd,dict(kind='operator_review',start=0,end=5,value='accepted',reason='test'))
    d=create_dataset(work,dict(split={'0':'train','1':'validation','2':'test'},allow_demo=True));directory=work/'datasets'/d['dataset_id']
    cfg={**DEFAULTS,'epochs':2,'batch_size':3,'scheduler':'none','augmentation':dict(horizontal_flip=False,brightness=0)};save_json(tmp_path/'cfg.json',cfg)
    def run(args):return subprocess.run([sys.executable,'-m','e2e.cli',*map(str,args)],capture_output=True,text=True,check=True)
    run(['train','--dataset',directory,'--output',tmp_path/'train','--config',tmp_path/'cfg.json'])
    model,cp=load_checkpoint(tmp_path/'train/best.pt');assert cp['dataset_version']==d['version'] and cp['optimizer'] and cp['epoch']>=0
    cfg.update(mode='finetune',checkpoint=str(tmp_path/'train/best.pt'),epochs=1);save_json(tmp_path/'fine.json',cfg)
    run(['train','--dataset',directory,'--output',tmp_path/'fine','--config',tmp_path/'fine.json'])
    run(['evaluate','--dataset',directory,'--checkpoints',tmp_path/'train/best.pt',tmp_path/'fine/best.pt','--output',tmp_path/'evaluation.json'])
    e=read_json(tmp_path/'evaluation.json');assert len(e['models'])==2 and not e['closed_loop_measured'] and len(e['models'][0]['predictions'])==6
    cfg.update(mode='resume',checkpoint=str(tmp_path/'train/last.pt'),epochs=3);save_json(tmp_path/'invalid.json',cfg)
    result=subprocess.run([sys.executable,'-m','e2e.cli','train','--dataset',str(directory),'--output',str(tmp_path/'invalid'),'--config',str(tmp_path/'invalid.json')],capture_output=True,text=True)
    assert result.returncode!=0 and 'unchanged epochs' in result.stderr
