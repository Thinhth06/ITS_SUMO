"""
Chạy script này để test WebSocket độc lập với SUMO:
  python test_ws.py
Mở http://localhost:9999 để test
"""
import asyncio
from contextlib import asynccontextmanager
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
import uvicorn

@asynccontextmanager
async def lifespan(app):
    print("[TEST] Server ready")
    yield

app = FastAPI(lifespan=lifespan)

@app.get("/", response_class=HTMLResponse)
async def index():
    return HTMLResponse("""<!DOCTYPE html>
<html><body>
<div id="log" style="font-family:monospace;font-size:14px"></div>
<script>
function log(msg, color='#fff'){
  document.getElementById('log').innerHTML += 
    '<div style="color:'+color+'">'+msg+'</div>';
}
log('Connecting...');
const ws = new WebSocket('ws://localhost:9999/ws');
ws.onopen = () => log('WS onopen fired', '#0f0');
ws.onmessage = e => log('MSG: ' + e.data, '#0ff');
ws.onclose = e => log('CLOSED code='+e.code+' reason='+e.reason, '#f80');
ws.onerror = e => log('ERROR: '+JSON.stringify(e), '#f00');
</script>
</body></html>""")

@app.websocket("/ws")
async def ws_ep(ws: WebSocket):
    await ws.accept()
    print("[WS] accepted")
    await asyncio.sleep(0)
    await ws.send_text('{"type":"hello","msg":"it works!"}')
    print("[WS] sent hello")
    try:
        while True:
            data = await ws.receive_text()
            print(f"[WS] recv: {data}")
    except WebSocketDisconnect:
        print("[WS] disconnected")

if __name__ == "__main__":
    print("Test WS server at http://localhost:9999")
    uvicorn.run("test_ws:app", host="0.0.0.0", port=9999, reload=False)
