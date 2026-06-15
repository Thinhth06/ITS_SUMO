# SUMO ITS Dashboard — road4lane v4

## Cấu trúc

```
sumo_its_project/
├── sumo_files/
│   ├── road4lane_net.xml       ← Map của bạn
│   ├── road4lane_rou.xml       ← Routes (1500 xe/chiều)
│   ├── road4lane_det.add.xml   ← 12 detectors
│   └── road4lane_its.sumocfg  ← Config + TraCI port 8813
├── server/
│   └── sumo_server.py          ← FastAPI + TraCI backend
└── dashboard/
    └── index.html              ← Web dashboard
```

## Cài đặt

```bash
pip install fastapi "uvicorn[standard]" traci
```

## Chạy

```bash
cd server
python sumo_server.py
# → http://localhost:8000
```

## Luồng sử dụng

1. **Kết nối** → nhấn ⚡ KẾT NỐI
2. **Set thời gian** mô phỏng (mặc định 300s)
3. **Nhấn ▶ BẮT ĐẦU** → SUMO chạy, tai nạn ngẫu nhiên xảy ra
4. **Sau khi xong** → modal phân tích tự hiện
5. **Xuất CSV** dữ liệu đầy đủ

## Phases

| Phase    | Mô tả                                      |
|----------|--------------------------------------------|
| WARMUP   | 10% đầu — xe warm up                       |
| BASELINE | Thu thập dữ liệu bình thường               |
| INCIDENT | Tai nạn random 1 chiều, chặn cả 2 làn      |
| FINISHED | Hiện modal phân tích + xuất CSV            |

## WebSocket commands (client → server)

```json
{"cmd":"start","duration":300}
{"cmd":"stop"}
{"cmd":"set_speed","direction":"A","speed_kmh":30}
```
