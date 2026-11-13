from pathlib import Path
import json
from playwright.sync_api import sync_playwright
root=Path(__file__).resolve().parents[1]
with sync_playwright() as p:
 b=p.chromium.launch(executable_path='/usr/bin/google-chrome',headless=True,args=['--no-sandbox','--disable-dev-shm-usage']);page=b.new_page(viewport={'width':1500,'height':1080});errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
 page.goto('http://127.0.0.1:8765');page.wait_for_function("document.querySelector('#sessionList').textContent.includes('DEMO')")
 state=page.evaluate("fetch('/api/state').then(r=>r.json())");sid=state['sessions'][0]['id'];page.locator('[data-tab=review]').click();page.locator('#reviewSession').select_option(sid);page.wait_for_function("document.querySelector('#rawImage').naturalWidth>0")
 page.locator('#seek').fill('8');page.locator('#seek').dispatch_event('input');page.locator('#timeline').click(position={'x':140,'y':70});page.wait_for_timeout(500)
 meta=json.loads(page.locator('#frameMeta').inner_text());assert isinstance(meta['original_stamp_ns'],str) and len(meta['original_stamp_ns'])==19
 assert meta['wheel_angle_deg'] is None and meta['speed_m_s'] is None
 page.screenshot(path=str(root/'docs/screenshots/review.png'),full_page=True)
 page.locator('[data-tab=evaluate]').click();page.locator('[data-report]').first.click();page.wait_for_timeout(500)
 assert not errors,errors
 result=dict(javascript_errors=errors,integer_nanoseconds_as_strings=True,source_stamp_example=meta['original_stamp_ns'],synthetic=True,vehicle_connection=False)
 (root/'docs/browser_final_check.json').write_text(json.dumps(result,indent=2));print(json.dumps(result));b.close()
