"""
sumo_server.py — road4lane, v5
Kiến trúc: TraCI chạy trong dedicated thread riêng.
Main async loop giao tiếp qua asyncio.Queue.
Điều này tránh hoàn toàn blocking event loop.
"""
import asyncio, json, os, subprocess, time, math, random, platform, threading, shutil
from pathlib import Path
from typing import Optional
from contextlib import asynccontextmanager
import uvicorn
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse

try:
    import traci
    TRACI_OK = True
except ImportError:
    print("[WARN] traci not found → DEMO mode")
    TRACI_OK = False

# ── Config ──────────────────────────────────────────────────────────────────
SUMO_PORT   = 8813
_IS_WIN     = platform.system() == "Windows"
SUMO_BIN    = "sumo.exe" if _IS_WIN else "sumo"

_here = Path(__file__).parent
_cfgs = [
    _here/"road4lane_its.sumocfg",           # cùng thư mục với server
    _here/"sumo_files"/"road4lane_its.sumocfg",
    _here.parent/"sumo_files"/"road4lane_its.sumocfg",
    _here/"road4lane.sumocfg",               # fallback config gốc
    _here/"sumo_files"/"road4lane.sumocfg",
]
SUMO_CFG    = next((str(p) for p in _cfgs if p.exists()), str(_cfgs[0]))

SIM_STEP    = 0.1          # bước mô phỏng nhỏ hơn → mượt hơn
BROADCAST_HZ= 20           # broadcast 20 frame/s
EDGES_A     = ["edge_A1","edge_A2"]
EDGES_B     = ["edge_B1","edge_B2"]
TOTAL_LEN   = 1000.0
NUM_LANES   = 2
SPEED_LIMIT = 13.89   # 50 km/h

DET_IDS = [
    "det_A_up_L0","det_A_up_L1","det_A_mid_L0","det_A_mid_L1","det_A_dn_L0","det_A_dn_L1",
    "det_B_up_L0","det_B_up_L1","det_B_mid_L0","det_B_mid_L1","det_B_dn_L0","det_B_dn_L1",
]

# ── Shared state ────────────────────────────────────────────────────────────
state = {
    "ready":False,"running":False,"sim_time":0.0,"sim_duration":300.0,
    "vehicles":[],"detectors":{},"incidents":[],"phase":"idle",
    "stats":{"speed_A":0.0,"speed_B":0.0,"flow_A":0,"flow_B":0,
             "density_A":0.0,"density_B":0.0,"count_A":0,"count_B":0,"occ_A":0.0,"occ_B":0.0},
    "speed_limit":{"A":50,"B":50},
    "sim_speed": 5,
    "history_normal":[],"history_incident":[],
    "incident_start_time":None,
}
clients: list[WebSocket] = []
sumo_proc: Optional[subprocess.Popen] = None
_state_lock = threading.Lock()

# ── TraCI helpers (all called from TraCI thread only) ───────────────────────
def _start_sumo_sync() -> bool:
    global sumo_proc
    sumo_bin = shutil.which(SUMO_BIN)
    local_sumo = _here.parent / "tools" / "sumo-1.27.0" / "PFiles" / "Eclipse" / "Sumo"
    if not sumo_bin and (local_sumo / "bin" / "sumo.exe").exists():
        os.environ["SUMO_HOME"] = str(local_sumo)
        sumo_bin = str(local_sumo / "bin" / "sumo.exe")
        print(f"[CONFIG] SUMO_HOME={local_sumo}")
    if _IS_WIN and not os.environ.get("SUMO_HOME"):
        for p in [Path(r"C:\Program Files (x86)\Eclipse\Sumo"),
                  Path(r"C:\Program Files\Eclipse\Sumo"), Path(r"C:\Sumo")]:
            candidate = p / "bin" / "sumo.exe"
            if candidate.exists():
                os.environ["SUMO_HOME"] = str(p)
                sumo_bin = str(candidate)
                print(f"[CONFIG] SUMO_HOME={p}"); break
    if not sumo_bin:
        print("[SUMO] sumo.exe not found. Install SUMO or add its bin folder to PATH.")
        return False
    cmd = [sumo_bin,"-c",SUMO_CFG,"--remote-port",str(SUMO_PORT),
           "--no-step-log","--step-length",str(SIM_STEP),
           "--collision.action","warn","--time-to-teleport","300"]
    # --threads bị bỏ: SUMO cảnh báo nó có bugs và không tăng tốc thực sự
    print(f"[SUMO] Config: {SUMO_CFG}  exists={Path(SUMO_CFG).exists()}")
    print(f"[SUMO] {' '.join(cmd)}")
    # cwd = thư mục chứa cfg → SUMO tìm net.xml, rou.xml, det.xml đúng chỗ
    sumo_cwd = str(Path(SUMO_CFG).parent)
    print(f"[SUMO] cwd={sumo_cwd}")
    sumo_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                 cwd=sumo_cwd)
    for attempt in range(24):
        time.sleep(0.5)
        if sumo_proc.poll() is not None:
            _,err=sumo_proc.communicate()
            print(f"[SUMO] Crashed!\n{err.decode(errors='replace')}"); return False
        try:
            traci.init(SUMO_PORT)
            print(f"[SUMO] TraCI connected (attempt {attempt+1})")
            return True
        except Exception as e:
            print(f"[SUMO] attempt {attempt+1}: {e}")
    return False

def _stop_sumo_sync():
    global sumo_proc
    try: traci.close()
    except: pass
    if sumo_proc:
        sumo_proc.terminate(); sumo_proc=None

def _edge_dir(edge):
    if edge in EDGES_A: return "A"
    if edge in EDGES_B: return "B"
    return None

def _get_vehicles():
    out=[]
    for vid in traci.vehicle.getIDList():
        try:
            edge=traci.vehicle.getRoadID(vid)
            d=_edge_dir(edge)
            if not d: continue
            pos=traci.vehicle.getLanePosition(vid)
            lane=traci.vehicle.getLaneIndex(vid)
            spd=traci.vehicle.getSpeed(vid)
            vtype=traci.vehicle.getTypeID(vid)
            color=traci.vehicle.getColor(vid)
            seg=0.0 if edge in ("edge_A1","edge_B1") else 500.0
            xn=(seg+pos)/TOTAL_LEN if d=="A" else 1.0-(seg+pos)/TOTAL_LEN
            out.append({"id":vid,"dir":d,"x":round(xn,4),"lane":min(lane,1),
                        "speed_ms":round(spd,2),"speed_kmh":round(spd*3.6,1),
                        "vtype":vtype,"length":traci.vehicle.getLength(vid),
                        "color":f"#{color[0]:02x}{color[1]:02x}{color[2]:02x}"})
        except: pass
    return out

_det_missing_logged = False
def _get_detectors():
    global _det_missing_logged
    data={}
    missing = []
    for did in DET_IDS:
        try:
            spd_ms = traci.inductionloop.getLastStepMeanSpeed(did)
            data[did]={
                "flow":    traci.inductionloop.getLastStepVehicleNumber(did),
                "speed_kmh": round(spd_ms * 3.6, 1) if spd_ms >= 0 else -1,
                "occupancy": round(traci.inductionloop.getLastStepOccupancy(did), 2)
            }
        except Exception as e:
            missing.append(did)
            data[did]={"flow":0,"speed_kmh":-1,"occupancy":0}
    if missing and not _det_missing_logged:
        _det_missing_logged = True
        print(f"[WARN] Detectors NOT FOUND in SUMO: {missing}")
        print(f"[WARN] Add road4lane_det.xml to sumocfg additional-files!")
    return data

# Bộ tích lũy flow — cửa sổ trượt 30s (300 bước × 0.1s)
_flow_window_A: list = []
_flow_window_B: list = []
FLOW_WINDOW_STEPS = 600   # 60s — mượt hơn

def _compute_stats(vehs, dets):
    global _flow_window_A, _flow_window_B
    vA=[v for v in vehs if v["dir"]=="A"]; vB=[v for v in vehs if v["dir"]=="B"]
    def asp(l): return round(sum(v["speed_kmh"] for v in l)/len(l),1) if l else 0.0
    def den(l): return round(len(l)/(TOTAL_LEN/1000)/NUM_LANES,1)

    # Flow đo tại detector DN (downstream) — sau tai nạn không có xe đến → flow giảm rõ nhất
    raw_A = sum(dets.get(f"det_A_dn_L{i}",{}).get("flow",0) for i in range(NUM_LANES))
    raw_B = sum(dets.get(f"det_B_dn_L{i}",{}).get("flow",0) for i in range(NUM_LANES))

    # Tích lũy vào cửa sổ trượt
    _flow_window_A.append(raw_A); _flow_window_B.append(raw_B)
    if len(_flow_window_A) > FLOW_WINDOW_STEPS: _flow_window_A.pop(0)
    if len(_flow_window_B) > FLOW_WINDOW_STEPS: _flow_window_B.pop(0)

    # Flow = tổng xe trong 30s (xe/30s) — đơn vị dễ đọc trên biểu đồ
    n_steps = len(_flow_window_A)
    window_s = n_steps * 0.1
    fA = round(sum(_flow_window_A) * 60 / window_s, 1) if window_s > 0 else 0  # xe/phút
    fB = round(sum(_flow_window_B) * 60 / window_s, 1) if window_s > 0 else 0  # xe/phút

    oA=sum(dets.get(f"det_A_mid_L{i}",{}).get("occupancy",0) for i in range(NUM_LANES))/NUM_LANES
    oB=sum(dets.get(f"det_B_mid_L{i}",{}).get("occupancy",0) for i in range(NUM_LANES))/NUM_LANES
    return {"speed_A":asp(vA),"speed_B":asp(vB),"flow_A":fA,"flow_B":fB,
            "density_A":den(vA),"density_B":den(vB),"count_A":len(vA),"count_B":len(vB),
            "occ_A":round(oA,2),"occ_B":round(oB,2)}

def _apply_incident(inc):
    d=inc["direction"]; pos_m=inc["position"]*TOTAL_LEN; blocked=inc.get("lanes_blocked",[0,1])
    if d=="A": edge,local=("edge_A1",pos_m) if pos_m<=500 else ("edge_A2",pos_m-500)
    else: edge,local=("edge_B1",pos_m) if pos_m<=500 else ("edge_B2",pos_m-500)
    lane_len = traci.lane.getLength(f"{edge}_0")
    stop_pos = min(max(local, 10.0), lane_len - 5.0)
    for vid in traci.vehicle.getIDList():
        try:
            if traci.vehicle.getRoadID(vid) != edge: continue
            lane_i = traci.vehicle.getLaneIndex(vid)
            if lane_i not in blocked: continue
            vpos = traci.vehicle.getLanePosition(vid)
            dist = stop_pos - vpos
            if 0 < dist < 120:  # xe đang tiến đến điểm tai nạn
                # Dừng xe bằng setSpeed thay vì setMaxSpeed lane (tránh crash SUMO)
                traci.vehicle.setSpeed(vid, 0.0)
                traci.vehicle.setSpeedMode(vid, 0)  # tắt mọi speed mode tự động
        except: pass
    # Không dùng lane.setMaxSpeed — gây SUMO crash ở một số version

def _clear_incidents_sync():
    for edge in EDGES_A+EDGES_B:
        for li in range(NUM_LANES):
            try: traci.lane.setMaxSpeed(f"{edge}_{li}",SPEED_LIMIT)
            except: pass

# ── Demo mode ───────────────────────────────────────────────────────────────
_demo_vehs=[]; _demo_t=0.0
DEMO_COLORS=["#3a7bd5","#e84393","#27ae60","#e67e22","#8e44ad","#c0392b","#f1c40f","#1abc9c"]

def _init_demo(n=40):
    global _demo_vehs
    _demo_vehs=[]
    for i in range(n):
        direction = "A" if i < n//2 else "B"
        lane = i % NUM_LANES           # phân bổ đều vào cả NUM_LANES làn
        _demo_vehs.append({
            "id": f"v{i}",
            "dir": direction,
            "x": (i/(n//2 if direction=="A" else n//2)) % 1.0,  # phân bổ đều trên đường
            "lane": lane,
            "speed_kmh": 45.0,
            "speed_ms": 12.5,
            "vtype": random.choice(["car","car","truck"]),
            "length": random.choice([5.0, 10.0]),
            "color": DEMO_COLORS[i % len(DEMO_COLORS)],
            "_spd": 0.0005 + random.random()*0.0004,
        })

def _step_demo():
    global _demo_t; _demo_t+=SIM_STEP
    for v in _demo_vehs:
        v["x"]=(v["x"]+v["_spd"])%1.0
        spd=45+8*math.sin(_demo_t/20+hash(v["id"])%8)
        for inc in state["incidents"]:
            if v["dir"]==inc["direction"] and abs(v["x"]-inc["position"])<0.08: spd=0
        v["speed_kmh"]=round(max(0,spd),1); v["speed_ms"]=round(v["speed_kmh"]/3.6,2)

def _demo_dets():
    data={}
    for did in DET_IDS:
        dir_char = "A" if "_A_" in did else "B"
        has_incident = any(inc["direction"]==dir_char for inc in state["incidents"])
        is_up  = "_up_"  in did
        is_mid = "_mid_" in did
        is_dn  = "_dn_"  in did
        if has_incident:
            # Upstream: tắc nghẽn — tốc độ thấp, occupancy cao
            # Downstream: thông — tốc độ cao, occupancy thấp
            if is_up:
                spd = round(max(0, random.gauss(8, 3)), 1)     # ~8 km/h
                occ = round(random.uniform(35, 55), 1)          # cao
                flw = random.randint(0, 2)
            elif is_mid:
                spd = round(max(0, random.gauss(12, 4)), 1)
                occ = round(random.uniform(20, 40), 1)
                flw = random.randint(0, 3)
            else:  # dn
                spd = round(random.gauss(42, 5), 1)            # downstream thông
                occ = round(random.uniform(3, 10), 1)           # thấp
                flw = random.randint(1, 4)
        else:
            spd = round(random.gauss(45, 4), 1)
            occ = round(random.uniform(8, 18), 1)
            flw = random.randint(1, 4)
        data[did]={"flow":flw,"speed_kmh":spd,"occupancy":occ}
    return data

# ── Analysis ─────────────────────────────────────────────────────────────────
def _build_analysis():
    inc_dir = state["incidents"][0]["direction"] if state["incidents"] else None
    inc_pos = state["incidents"][0]["position"]  if state["incidents"] else 0

    # Chọn key theo chiều bị tai nạn (nếu không có incident → so sánh cả 2)
    if inc_dir == "A":
        spd_key = "speed_A"; flow_key = "flow_A"; den_key = "density_A"; occ_key = "occ_A"
    elif inc_dir == "B":
        spd_key = "speed_B"; flow_key = "flow_B"; den_key = "density_B"; occ_key = "occ_B"
    else:
        spd_key = "speed_A"; flow_key = "flow_A"; den_key = "density_A"; occ_key = "occ_A"

    KEYS = [spd_key, flow_key, den_key, occ_key,
            "speed_A","speed_B","flow_A","flow_B","density_A","density_B","occ_A","occ_B"]

    def avg(lst, k): return round(sum(r[k] for r in lst if k in r) / len(lst), 3) if lst else 0
    def col(lst):
        if not lst: return {k: 0 for k in KEYS} | {"samples": 0}
        return {k: avg(lst, k) for k in KEYS} | {"samples": len(lst)}

    n   = col(state["history_normal"])
    inc = col(state["history_incident"])

    # Đảm bảo mọi key tồn tại dù history rỗng
    for k in KEYS:
        n.setdefault(k, 0)
        inc.setdefault(k, 0)

    # So sánh % chỉ trên chiều bị tai nạn
    cmp_keys = [spd_key, flow_key, den_key, occ_key]
    cmp = {k: round((inc[k]-n[k])/n[k]*100, 1) if n.get(k,0)!=0 else 0
           for k in cmp_keys}

    inc_t   = state.get("incident_start_time")
    det_t   = state.get("incident_detected_time")
    delay   = round(det_t - inc_t, 1) if (inc_t is not None and det_t is not None) else None
    return {
        "incident_dir": inc_dir or "—",
        "incident_pos": inc_pos,
        "affected_dir": inc_dir,
        "spd_key": spd_key, "flow_key": flow_key,
        "den_key": den_key, "occ_key": occ_key,
        "incident_time":   round(inc_t,1) if inc_t is not None else None,
        "detection_time":  round(det_t,1) if det_t is not None else None,
        "detection_delay": delay,
        "normal":  n,
        "incident": inc,
        "comparison": cmp,
        "history_normal":   state["history_normal"][-300:],
        "history_incident": state["history_incident"][-300:],
    }

# ── Async broadcast ──────────────────────────────────────────────────────────
async def _broadcast(msg:dict):
    if not clients: return
    txt=json.dumps(msg)
    dead=[]
    for ws in list(clients):
        try: await ws.send_text(txt)
        except: dead.append(ws)
    for ws in dead:
        if ws in clients: clients.remove(ws)

# ── Main simulation coroutine ────────────────────────────────────────────────
async def _sim_loop():
    duration=state["sim_duration"]
    warmup=min(90.0,duration*0.35)  # 35% để xe điền đầy đường và qua detector
    inc_time=warmup+(duration-warmup)*random.uniform(0.2,0.5)
    inc_triggered=False
    state.update({"phase":"warmup","history_normal":[],"history_incident":[],"incidents":[],
                  "incident_start_time":None,"incident_detected_time":None})
    global _flow_window_A, _flow_window_B, _det_missing_logged
    _flow_window_A=[]; _flow_window_B=[]; _det_missing_logged=False

    print(f"[SIM] duration={duration}s warmup={warmup:.0f}s incident@{inc_time:.0f}s")
    await _broadcast({"type":"sim_phase","phase":"warmup","incident_at":round(inc_time,1),"duration":duration})

    loop=asyncio.get_event_loop()
    step=0
    # Broadcast mỗi BROADCAST_HZ lần/giây thực: với SIM_STEP=0.1, mỗi step=0.1s sim
    # Để broadcast ~20fps thực thì gửi mỗi 1 step (vì sleep=SIM_STEP/speedup)
    BCAST_EVERY = max(1, int(round(1.0 / (BROADCAST_HZ * SIM_STEP))))
    use_traci=TRACI_OK and state.get("_traci_ok",False)

    # _ensure_multilane đã bỏ — route file có flow tự động phân làn

    try:
        while state["sim_time"]<duration and state["running"]:
            t0=loop.time()

            if use_traci:
                try:
                    def _step_all():
                        traci.simulationStep()
                        t2=round(traci.simulation.getTime(),1)
                        v2=_get_vehicles()
                        d2=_get_detectors()
                        return t2,v2,d2
                    sim_time,vehs,dets=await loop.run_in_executor(None,_step_all)
                    state["sim_time"]=sim_time

                    # Chỉ apply incident sau 3s để xe kịp react tự nhiên trước
                    _inc_st = state.get("incident_start_time")
                    if _inc_st and (sim_time - _inc_st) >= 3.0 and step%10==0:
                        for inc in list(state["incidents"]):
                            await loop.run_in_executor(None,_apply_incident,inc)
                except Exception as e:
                    err_str = str(e)
                    print(f"[SUMO] Step error: {err_str}")
                    # Nếu SUMO tự dừng (FatalTraCIError / connection closed) → kết thúc gracefully
                    if "connection" in err_str.lower() or "closed" in err_str.lower() or "fatal" in err_str.lower():
                        print("[SUMO] SUMO ended — finishing simulation gracefully")
                        break
                    break
            else:
                _step_demo()
                state["sim_time"]=round(_demo_t,1)
                vehs=list(_demo_vehs); dets=_demo_dets()

            stats=_compute_stats(vehs,dets)
            state.update({"vehicles":vehs,"detectors":dets,"stats":stats})
            t=state["sim_time"]

            if t>=warmup and state["phase"]=="warmup":
                state["phase"]="normal"
                await _broadcast({"type":"sim_phase","phase":"normal"})

            if t>=inc_time and not inc_triggered:
                inc_triggered=True
                direction=random.choice(["A","B"])
                position=round(random.uniform(0.3,0.7),2)
                inc={"id":f"inc_{int(time.time())}","direction":direction,
                     "position":position,"lanes_blocked":[0,1],"time":t}
                state["incidents"].append(inc)
                state["incident_start_time"]=t
                state["incident_detected_time"]=None
                state["phase"]="incident"
                # Reset flow window để tránh data bình thường lẫn vào phase incident
                _flow_window_A.clear(); _flow_window_B.clear()
                await _broadcast({"type":"sim_phase","phase":"incident","incident":inc,
                                  "incident_time": round(t,1)})
                print(f"[SIM] Incident: chiều {direction} @ {position*100:.0f}%")
                # Log sample detector data để debug
                for _did in [f"det_{direction}_up_L0", f"det_{direction}_dn_L0"]:
                    _d = state["detectors"].get(_did, {})
                    print(f"[DET] {_did}: spd={_d.get('speed_kmh','?')} occ={_d.get('occupancy','?')}")

            record={"t":t,**stats}
            if state["phase"] in ("warmup","normal"):
                state["history_normal"].append(record)
            elif state["phase"]=="incident":
                state["history_incident"].append(record)

            if step%BCAST_EVERY==0:
                await _broadcast({"type":"sim_update","sim_time":t,"duration":duration,
                                   "phase":state["phase"],"vehicles":vehs,"detectors":dets,
                                   "stats":stats,"incidents":state["incidents"],
                                   "speed_limit":state["speed_limit"]})
            step+=1
            # Không sleep nếu step quá nhanh — chỉ yield để event loop xử lý
            elapsed = loop.time()-t0
            # sim_speed: bội số tốc độ (1=realtime, 10=nhanh 10×, 0=không sleep)
            spd = max(1, state.get("sim_speed", 5))
            target = SIM_STEP / spd          # thời gian thực cho mỗi bước sim
            sleep_t = max(0, target - elapsed)
            if sleep_t > 0.0005:
                await asyncio.sleep(sleep_t)
            else:
                await asyncio.sleep(0)       # yield event loop không sleep
    finally:
        state["running"]=False; state["phase"]="finished"
        await _broadcast({"type":"sim_finished","analysis":_build_analysis()})
        print("[SIM] Finished.")

# ── FastAPI ──────────────────────────────────────────────────────────────────
async def _init_sumo_background():
    """Khởi động SUMO trong nền — không block uvicorn."""
    loop = asyncio.get_event_loop()
    print("[BG] Starting SUMO in background thread...")
    ok = await loop.run_in_executor(None, _start_sumo_sync)
    state["ready"]    = True
    state["_traci_ok"]= ok
    if ok:
        print("[BG] SUMO ready. Clients will be notified.")
        # Thông báo cho tất cả client đang chờ
        await _broadcast({"type":"server_ready","traci_ok":True})
    else:
        print("[BG] SUMO failed → demo mode")
        _init_demo(40)
        await _broadcast({"type":"server_ready","traci_ok":False})

@asynccontextmanager
async def lifespan(_app):
    # KHÔNG await start_sumo — để uvicorn accept connections ngay
    state["ready"]     = False
    state["_traci_ok"] = False
    if TRACI_OK:
        # Chạy nền, uvicorn sẵn sàng ngay lập tức
        asyncio.create_task(_init_sumo_background())
    else:
        _init_demo(40)
        state["ready"]     = True
        state["_traci_ok"] = False
    yield
    loop = asyncio.get_event_loop()
    if TRACI_OK and state.get("_traci_ok"):
        await loop.run_in_executor(None, _stop_sumo_sync)

app=FastAPI(title="SUMO ITS",lifespan=lifespan)

@app.get("/api/status")
async def api_status():
    return JSONResponse({"ready":state["ready"],"running":state["running"],
                         "phase":state["phase"],"sim_time":state["sim_time"],
                         "traci":TRACI_OK,"traci_ok":state.get("_traci_ok",False),
                         "vehicles":len(state["vehicles"]),"sumo_cfg":SUMO_CFG})

@app.post("/api/start")
async def api_start(body:dict):
    global _demo_t
    if state["running"]: return JSONResponse({"ok":False,"msg":"Already running"})
    duration=float(body.get("duration",300))
    state.update({"sim_duration":duration,"sim_time":0.0,"running":True,"phase":"warmup","incidents":[],
                  "incident_start_time":None,"incident_detected_time":None})
    loop=asyncio.get_event_loop()
    if TRACI_OK and state.get("_traci_ok"):
        print("[START] Restarting SUMO...")
        await loop.run_in_executor(None,_stop_sumo_sync)
        await asyncio.sleep(1.5)
        ok=await loop.run_in_executor(None,_start_sumo_sync)
        state["_traci_ok"]=ok
        if not ok: _init_demo(40)
    else:
        _demo_t=0.0; _init_demo(40)
    asyncio.create_task(_sim_loop())
    return JSONResponse({"ok":True,"duration":duration})
@app.post("/api/stop")
async def api_stop():
    state["running"]=False
    return JSONResponse({"ok":True})

@app.post("/api/incident/clear")
async def api_clear_inc():
    loop=asyncio.get_event_loop()
    if TRACI_OK and state.get("_traci_ok"):
        await loop.run_in_executor(None,_clear_incidents_sync)
    state["incidents"]=[]
    return JSONResponse({"ok":True})

@app.post("/api/incident/detected")
async def api_incident_detected(req:Request):
    data = await req.json()
    t = data.get("sim_time")
    if state.get("incident_detected_time") is None and state.get("incident_start_time") is not None:
        state["incident_detected_time"] = t
        delay = round(t - state["incident_start_time"], 1)
        print(f"[DETECT] Phát hiện tai nạn t={t}s delay={delay}s")
    return JSONResponse({"ok":True})

@app.post("/set_sim_speed")
async def set_sim_speed(req:Request):
    data=await req.json()
    spd=max(1,min(200,int(data.get("speed",5))))
    state["sim_speed"]=spd
    print(f"[API] sim_speed={spd}x")
    return JSONResponse({"ok":True,"sim_speed":spd})

@app.post("/api/control")
async def api_control(body:dict):
    d=body.get("direction","A"); speed=float(body.get("speed_kmh",50))
    state["speed_limit"][d]=speed
    if TRACI_OK and state.get("_traci_ok"):
        loop=asyncio.get_event_loop()
        edges=EDGES_A if d=="A" else EDGES_B
        def _set():
            for e in edges:
                for li in range(NUM_LANES):
                    try: traci.lane.setMaxSpeed(f"{e}_{li}",speed/3.6)
                    except: pass
        await loop.run_in_executor(None,_set)
    return JSONResponse({"ok":True})

@app.websocket("/ws")
async def ws_endpoint(ws:WebSocket):
    await ws.accept()
    clients.append(ws)
    print(f"[WS] +client total={len(clients)}")
    await asyncio.sleep(0.05)   # nhường event loop
    try:
        await ws.send_text(json.dumps({
            "type":"connected","ready":state["ready"],
            "running":state["running"],"phase":state["phase"],
            "traci":TRACI_OK,"traci_ok":state.get("_traci_ok",False),
        }))
        print("[WS] Sent connected")
    except Exception as e:
        print(f"[WS] send_connected failed: {e}")
        if ws in clients: clients.remove(ws)
        return
    try:
        while True:
            raw=await ws.receive_text()
            msg=json.loads(raw)
            cmd=msg.get("cmd","")
            if   cmd=="start":            await api_start(msg)
            elif cmd=="stop":             await api_stop()
            elif cmd=="clear_incidents":  await api_clear_inc()
            elif cmd=="set_speed":        await api_control(msg)
            elif cmd=="incident_detected":
                t_det = msg.get("sim_time")
                if t_det and state.get("incident_detected_time") is None and state.get("incident_start_time") is not None:
                    state["incident_detected_time"] = t_det
                    delay = round(t_det - state["incident_start_time"], 1)
                    print(f"[DETECT via WS] t={t_det}s delay={delay}s")
    except WebSocketDisconnect:
        if ws in clients: clients.remove(ws)
        print(f"[WS] -client total={len(clients)}")
    except Exception as e:
        print(f"[WS] error: {e}")
        if ws in clients: clients.remove(ws)

@app.get("/",response_class=HTMLResponse)
async def dashboard():
    for p in [_here/"dashboard"/"index.html",_here.parent/"dashboard"/"index.html"]:
        if p.exists(): return HTMLResponse(p.read_text(encoding="utf-8"))
    return HTMLResponse("<h1>Place dashboard/index.html next to server/</h1>")

if __name__=="__main__":
    print("="*60)
    print("  SUMO ITS Dashboard — road4lane v5")
    print(f"  TraCI:    {'OK' if TRACI_OK else 'NOT FOUND (demo)'}")
    print(f"  CFG:      {SUMO_CFG}  exists={Path(SUMO_CFG).exists()}")
    print(f"  URL:      http://localhost:8000")
    print("="*60)
    import webbrowser, threading
    threading.Timer(1.5, lambda: webbrowser.open("http://localhost:8000")).start()
    uvicorn.run("sumo_server:app",host="0.0.0.0",port=8000,reload=False)
