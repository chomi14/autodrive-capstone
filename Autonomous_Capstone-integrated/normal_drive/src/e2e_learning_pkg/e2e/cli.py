import argparse
import json
from pathlib import Path
import sys
import traceback

from .core import import_session,create_dataset,read_json,save_json


def main():
    p=argparse.ArgumentParser(description='Normal-driving session workbench. No vehicle actuation commands.')
    sub=p.add_subparsers(dest='command',required=True)
    a=sub.add_parser('import');a.add_argument('source');a.add_argument('target')
    a=sub.add_parser('dataset');a.add_argument('--work',required=True);a.add_argument('--config',required=True)
    a=sub.add_parser('train');a.add_argument('--dataset',required=True);a.add_argument('--output',required=True);a.add_argument('--config',required=True)
    a=sub.add_parser('evaluate');a.add_argument('--dataset',required=True);a.add_argument('--checkpoints',nargs='+',required=True);a.add_argument('--output',required=True);a.add_argument('--device',default='cpu')
    a=sub.add_parser('serve');a.add_argument('--work',default='work');a.add_argument('--port',type=int,default=8765)
    a=sub.add_parser('model-import');a.add_argument('source');a.add_argument('destination')
    a=sub.add_parser('demo');a.add_argument('output');a.add_argument('--frames',type=int,default=18)
    p.add_argument('--progress-file')
    args=p.parse_args()
    def progress(current,total):
        if args.progress_file:save_json(args.progress_file,dict(current=current,total=total))
    if args.command=='import':
        result=import_session(args.source,args.target,progress);print(json.dumps(result['stats']))
    elif args.command=='dataset':print(json.dumps(create_dataset(args.work,read_json(args.config),progress)['stats']))
    elif args.command=='train':
        from .learning import train
        train(args.dataset,args.output,read_json(args.config))
    elif args.command=='evaluate':
        from .learning import evaluate
        result=evaluate(args.dataset,args.checkpoints,args.output,args.device);print(json.dumps([{k:v for k,v in m.items() if k not in ('predictions','worst_frames')} for m in result['models']]))
    elif args.command=='serve':
        from .server import serve
        serve(args.work,args.port)
    elif args.command=='model-import':
        from .learning import import_model
        print(json.dumps(import_model(args.source,args.destination)))
    elif args.command=='demo':
        from .synthetic import make_demo
        print(json.dumps(make_demo(args.output,args.frames)))

if __name__=='__main__':main()
