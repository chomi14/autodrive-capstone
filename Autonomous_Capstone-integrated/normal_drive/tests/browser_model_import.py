from pathlib import Path
import json,time
from playwright.sync_api import sync_playwright
root=Path(__file__).resolve().parents[1];cp=json.loads((root/'docs/browser_validation.json').read_text())['models'][0]['checkpoint']
with sync_playwright() as p:
 b=p.chromium.launch(executable_path='/usr/bin/google-chrome',headless=True,args=['--no-sandbox','--disable-dev-shm-usage']);page=b.new_page(viewport={'width':1500,'height':1080});errors=[];page.on('pageerror',lambda e:errors.append(str(e)));page.goto('http://127.0.0.1:8765');page.wait_for_function("document.querySelector('#sessionList').textContent.includes('DEMO')")
 def state():return page.evaluate("fetch('/api/state').then(r=>r.json())")
 page.locator('[data-tab=train]').click()
 results=[]
 for source,expected in [(cp,'completed'),(str(root/'src/skku_track_drive_pkg/models/best_v3.pt'),'failed')]:
  prior={j['id'] for j in state()['jobs']};page.locator('#modelImportPath').fill(source);page.locator('#modelImport').click()
  deadline=time.monotonic()+25
  while time.monotonic()<deadline:
   jobs=[j for j in state()['jobs'] if j['id'] not in prior and j['kind']=='model-import']
   if jobs and jobs[0]['state'] in ('completed','failed'):break
   page.wait_for_timeout(200)
  assert jobs and jobs[0]['state']==expected,jobs
  if expected=='failed':assert 'YOLO' in jobs[0]['log']
  results.append(dict(input='E2E PilotNet' if expected=='completed' else 'actual repository YOLO .pt',state=jobs[0]['state']))
 page.locator('[data-tab=review]').click();page.locator('#reviewSession').select_option(state()['sessions'][0]['id']);page.wait_for_function("document.querySelector('#rawImage').naturalWidth>0")
 page.locator('#rangeStart').fill('1');page.locator('#rangeEnd').fill('3');page.locator('#playRange').click();page.wait_for_timeout(700)
 assert page.locator('#seek').input_value()=='3'
 page.screenshot(path=str(root/'docs/screenshots/review.png'),full_page=True)
 assert not errors,errors
 result=dict(synthetic=True,model_import=results,range_playback=True,javascript_errors=errors)
 (root/'docs/model_import_validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result));b.close()
