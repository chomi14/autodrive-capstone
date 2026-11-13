"""Actual Chromium GUI workflow against synthetic bags; no vehicle interaction."""
import json
from pathlib import Path
import time
import os
from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[1];work=Path(os.environ.get('E2E_VALIDATION_WORK',str(ROOT/'work'))).resolve();screens=ROOT/'docs/screenshots';screens.mkdir(exist_ok=True)
with sync_playwright() as p:
    browser=p.chromium.launch(executable_path='/usr/bin/google-chrome',headless=True,args=['--no-sandbox','--disable-dev-shm-usage'])
    page=browser.new_page(viewport={'width':1500,'height':1080});errors=[]
    page.on('pageerror',lambda e:errors.append(str(e)))
    page.goto('http://127.0.0.1:8765');page.wait_for_function("window.document.querySelector('#sessionList').textContent.length>5")
    def data():return page.evaluate("fetch('/api/state').then(r=>r.json())")
    def wait_job(kind,timeout=90):
        deadline=time.time()+timeout
        while time.time()<deadline:
            s=data();jobs=[j for j in s['jobs'] if j['kind']==kind]
            if jobs and jobs[0]['state'] in ('completed','failed'):
                if jobs[0]['state']=='failed':raise AssertionError(jobs[0]['log'])
                return jobs[0]
            page.wait_for_timeout(400)
        raise TimeoutError(kind)
    # Import through the actual UI (each run remains separate).
    for folder in sorted((work/'demo').iterdir()):
        run=json.loads((folder/'manifest.json').read_text())['run_id']
        if any(x['run_id']==run and x['id']!='demo1' for x in data()['sessions']):continue
        page.locator('#importPath').fill(str(folder));page.locator('#importButton').click();wait_job('import');page.wait_for_timeout(3200)
    s=data();ids=[x['id'] for x in s['sessions'] if x['id']!='demo1' and x['manifest'].get('demo')];assert len(ids)==3
    page.locator('[data-tab=review]').click()
    for sid in ids:
        page.locator('#reviewSession').select_option(sid);page.wait_for_timeout(500)
        page.locator('#rangeStart').fill('0');page.locator('#rangeEnd').fill('17');page.locator('#reason').fill('Synthetic DEMO browser verification; no driving success claim')
        page.locator('#accept').click();page.wait_for_timeout(350)
    page.locator('#seek').fill('5');page.locator('#seek').dispatch_event('input');page.wait_for_timeout(400)
    page.locator('#editLabel').fill('2');page.locator('#labelEdit').click();page.wait_for_timeout(300)
    page.locator('#undo').click();page.wait_for_timeout(350)
    page.locator('#exclude').click();page.wait_for_timeout(300);page.locator('#restore').click();page.wait_for_timeout(300)
    page.locator('summary').filter(has_text='시간 정렬 검토').click()
    page.locator('#alignmentNote').fill('Direct source match preserved; synthetic receipt delayed by 45 ms')
    page.locator('#alignment').click(force=True) if page.locator('#alignment').is_visible() else None
    page.locator('#rawImage').wait_for(state='visible');page.wait_for_function("document.querySelector('#rawImage').naturalWidth>0 && document.querySelector('#cropImage').naturalWidth>0")
    page.screenshot(path=str(screens/'review.png'),full_page=True)
    page.locator('[data-tab=dataset]').click()
    for sid,split in zip(ids,['train','validation','test']):page.locator(f'[data-split="{sid}"]').select_option(split)
    page.locator('#allowDemo').check();page.locator('#buildDataset').click();wait_job('dataset');page.wait_for_timeout(3200)
    from e2e.core import create_dataset
    latest=next(j for j in data()['jobs'] if j['kind']=='dataset')
    expected=create_dataset(work,latest['config'])
    s=data();assert s['datasets'];dataset=expected['dataset_id'];assert next(d for d in s['datasets'] if d['id']==dataset)['stats']['counts']=={'train':18,'validation':18,'test':18}
    page.screenshot(path=str(screens/'dataset.png'),full_page=True)
    page.locator('[data-tab=train]').click();page.locator('#trainDataset').select_option(dataset);page.locator('#epochs').fill('20');page.locator('#batchSize').fill('6')
    page.locator('summary').filter(has_text='고급 설정').click()
    advanced=page.locator('#advanced');cfg=json.loads(advanced.input_value());cfg['early_stopping']=0;cfg['scheduler']='none';advanced.fill(json.dumps(cfg))
    page.locator('#startTraining').click()
    # Stop after a real checkpoint appears, then resume with the full optimizer state.
    deadline=time.time()+45
    while time.time()<deadline:
        s=data();t=next((t for t in s['training'] if t.get('metrics')),None)
        if t:break
        page.wait_for_timeout(200)
    assert t and t['metrics']['epoch']>=1
    page.locator('#stopTraining').click();wait_job('train');page.wait_for_timeout(3200)
    s=data();first=s['training'][0];assert first['metrics']['state']=='stopped', first
    last=next(c for c in first['checkpoints'] if c.endswith('last.pt'))
    page.locator('#mode').select_option('resume');page.locator('#checkpoint').select_option(last);page.locator('#startTraining').click();wait_job('train',120);page.wait_for_timeout(3200)
    s=data();resumed=next(t for t in s['training'] if t['id']!=first['id']);assert resumed['metrics']['epoch']==20
    page.screenshot(path=str(screens/'training.png'),full_page=True)
    page.locator('[data-tab=evaluate]').click();page.locator('#evalDataset').select_option(dataset)
    a=next(c for c in resumed['checkpoints'] if c.endswith('last.pt'));b=last
    page.locator('#modelA').select_option(a);page.locator('#modelB').select_option(b);page.locator('#evaluateButton').click();wait_job('evaluate');page.wait_for_timeout(3200)
    s=data();report=s['evaluations'][0]['report'];assert len(report['models'])==2 and not report['closed_loop_measured']
    page.screenshot(path=str(screens/'evaluation.png'),full_page=True)
    page.locator('[data-worst]').first.click();page.wait_for_timeout(600);assert page.locator('#review').evaluate("e=>e.classList.contains('active')")
    assert not errors,errors
    result=dict(browser='installed Google Chrome via Playwright',synthetic=True,sessions=ids,dataset=dataset,
        train_stop_epoch=first['metrics']['epoch'],resumed_epoch=resumed['metrics']['epoch'],
        models=[{k:v for k,v in m.items() if k not in ('predictions','worst_frames')} for m in report['models']],javascript_errors=errors)
    (ROOT/'docs/browser_validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));browser.close()
