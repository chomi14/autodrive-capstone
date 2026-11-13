"""Actual GPU training/resume/evaluation; synthetic data only; no ROS/hardware IO."""
import json,os,signal,subprocess,sys,time
from pathlib import Path
import torch
from e2e.core import read_json,save_json
from e2e.learning import DEFAULTS,load_checkpoint,evaluate
root=Path(__file__).resolve().parents[1];gui=read_json(root/'docs/browser_validation.json');work=root/'work/final_gui';dataset=work/'datasets'/gui['dataset'];out=root/'work'/('gpu_validation_'+str(time.time_ns()));out.mkdir()
assert torch.cuda.is_available()
config={**DEFAULTS,'device':'cuda:0','epochs':8,'batch_size':6,'early_stopping':0,'scheduler':'none'};save_json(out/'config.json',config)
log=(out/'initial.log').open('w');p=subprocess.Popen([sys.executable,'-m','e2e.cli','train','--dataset',str(dataset),'--output',str(out/'initial'),'--config',str(out/'config.json')],stdout=log,stderr=subprocess.STDOUT)
deadline=time.monotonic()+90
while time.monotonic()<deadline:
 if (out/'initial/metrics.json').exists():
  p.send_signal(signal.SIGTERM);break
 if p.poll() is not None:raise RuntimeError((out/'initial.log').read_text())
 time.sleep(.03)
p.wait(timeout=60);log.close()
if p.returncode:raise RuntimeError((out/'initial.log').read_text())
initial=read_json(out/'initial/metrics.json');assert initial['state']=='stopped',initial
config.update(mode='resume',checkpoint=str(out/'initial/last.pt'));save_json(out/'resume.json',config)
log=(out/'resume.log').open('w');p=subprocess.run([sys.executable,'-m','e2e.cli','train','--dataset',str(dataset),'--output',str(out/'resumed'),'--config',str(out/'resume.json')],stdout=log,stderr=subprocess.STDOUT);log.close()
if p.returncode:raise RuntimeError((out/'resume.log').read_text())
resumed=read_json(out/'resumed/metrics.json');assert resumed['epoch']==8
os.environ.setdefault('CUBLAS_WORKSPACE_CONFIG',':4096:8')
evaluation=evaluate(dataset,[out/'initial/last.pt',out/'resumed/best.pt'],out/'evaluation.json','cuda:0')
model,meta=load_checkpoint(out/'resumed/last.pt','cuda:0');assert next(model.parameters()).is_cuda
result=dict(synthetic=True,gpu=torch.cuda.get_device_name(0),driver=subprocess.run(['nvidia-smi','--query-gpu=driver_version','--format=csv,noheader'],capture_output=True,text=True).stdout.strip(),torch=str(torch.__version__),cuda_build=torch.version.cuda,stopped_epoch=initial['epoch'],resumed_epoch=resumed['epoch'],dataset_version=meta['dataset_version'],output=str(out),models=[{k:v for k,v in m.items() if k not in ('predictions','worst_frames')} for m in evaluation['models']],peak_allocated_bytes=torch.cuda.max_memory_allocated())
save_json(root/'docs/gpu_validation.json',result)
actual=dict(cuda_available=True,cuda_build=torch.version.cuda,gpu=result['gpu'],driver=result['driver'],camera_device_exists=os.path.exists('/dev/video2'),nvidia_device_exists=os.path.exists('/dev/nvidia0'),execution='outside sandbox; actual host',gpu_validation=True)
save_json(root/'docs/host_devices.json',actual);print(json.dumps(result,indent=2))
