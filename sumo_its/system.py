import traci
import pandas as pd
import random
from cali import CaliforniaAlgorithm

sumo_cmd = [
    # "sumo-gui",
    # "-c", "C:\Program Files (x86)\Eclipse\sumo_files\road4lane.rou.xml",
    # # "--start"
    "sumo-gui",
    "-c", r"C:\Users\Thinh\Downloads\sumo_its\simulation.sumocfg", 
    # "--start",
    "--time-to-teleport", "-1"
]


def run_simulation():
    traci.start(sumo_cmd)
    cal_A = CaliforniaAlgorithm()  # Detector cho chiều A
    cal_B = CaliforniaAlgorithm()  # Detector cho chiều B
    data  = []

    # Upstream và downstream cho từng chiều
    edge_A_up   = "edge_A1"
    edge_A_down = "edge_A2"
    edge_B_up   = "edge_B1"
    edge_B_down = "edge_B2"

    # Chọn ngẫu nhiên chiều bị tai nạn
    incident_direction = random.choice(["A", "B"])
    incident_time      = random.randint(60, 200)
    incident_done      = False

    print(f"🚦 Hệ thống giám sát 4 làn 2 chiều đang chạy...")
    print(f"⏳ Tai nạn sẽ xảy ra bất ngờ ở chiều {incident_direction}!")

    try:
        for step in range(2400):
            traci.simulationStep()
            t = traci.simulation.getTime()

            # Tạo tai nạn ngẫu nhiên
            if t >= incident_time and not incident_done:

                if incident_direction == "A":
                    edge_incident = edge_A_up
                else:
                    edge_incident = edge_B_up

                vehicles = traci.edge.getLastStepVehicleIDs(edge_incident)

                if len(vehicles) >= 2:

                    car1 = vehicles[0]
                    car2 = vehicles[1]

                    try:
                        # Ép 2 xe sang 2 lane khác nhau
                        traci.vehicle.changeLane(car1, 0, 1)
                        traci.vehicle.changeLane(car2, 1, 1)

                        # Lấy chiều dài edge
                        edge_length = traci.lane.getLength(f"{edge_incident}_0")

                        # Random vị trí tai nạn
                        # tránh quá gần đầu/cuối đường
                        accident_pos = random.uniform(50, edge_length - 50)

                        # Dừng xe tạo blockage thật
                        traci.vehicle.setStop(
                            vehID=car1,
                            edgeID=edge_incident,
                            pos=accident_pos,
                            laneIndex=0,
                            duration=9999
                        )

                        traci.vehicle.setStop(
                            vehID=car2,
                            edgeID=edge_incident,
                            pos=accident_pos,
                            laneIndex=1,
                            duration=9999
                        )

                        # Khóa đổi làn cho xe tai nạn
                        traci.vehicle.setLaneChangeMode(car1, 0)
                        traci.vehicle.setLaneChangeMode(car2, 0)

                        # Đổi màu cho dễ nhìn
                        traci.vehicle.setColor(car1, (255, 0, 0))
                        traci.vehicle.setColor(car2, (255, 0, 0))

                        # Khóa tốc độ
                        traci.vehicle.setSpeedMode(car1, 0)
                        traci.vehicle.setSpeedMode(car2, 0)

                        incident_done = True
                        print(f"\nTAI NẠN xảy ra lúc t={t}s tại chiều {incident_direction}!")
                        print(f"   Vị trí: {edge_incident} (làn 0 và làn 1)\n")
                    except Exception as e:
                        print(f"Không tạo được tai nạn: {e}")
            # Thu thập dữ liệu chiều A
            speed_A_up   = traci.edge.getLastStepMeanSpeed(edge_A_up)
            speed_A_down = traci.edge.getLastStepMeanSpeed(edge_A_down)
            occ_A_up = (
                traci.lane.getLastStepOccupancy(f"{edge_A_up}_0") +
                traci.lane.getLastStepOccupancy(f"{edge_A_up}_1")
            ) / 2

            occ_A_down = (
                traci.lane.getLastStepOccupancy(f"{edge_A_down}_0") +
                traci.lane.getLastStepOccupancy(f"{edge_A_down}_1")
            ) / 2

            # Thu thập dữ liệu chiều B
            speed_B_up   = traci.edge.getLastStepMeanSpeed(edge_B_up)
            speed_B_down = traci.edge.getLastStepMeanSpeed(edge_B_down)
            occ_B_up = (
                traci.lane.getLastStepOccupancy(f"{edge_B_up}_0") +
                traci.lane.getLastStepOccupancy(f"{edge_B_up}_1")
            ) / 2

            occ_B_down = (
                traci.lane.getLastStepOccupancy(f"{edge_B_down}_0") +
                traci.lane.getLastStepOccupancy(f"{edge_B_down}_1")
            ) / 2

            # Phát hiện sự cố từng chiều
            incident_A, msg_A = cal_A.detect(occ_A_up, occ_A_down, speed_A_up, speed_A_down, t)
            incident_B, msg_B = cal_B.detect(occ_B_up, occ_B_down, speed_B_up, speed_B_down, t)

            # Thông báo chi tiết làn nào bị tắc
            if incident_A:
                print(f" [ALERT] CHIỀU A BỊ TẮC!")
                print(f"   ├─ Làn A-0 ({edge_A_up}_0): tắc nghẽn")
                print(f"   ├─ Làn A-1 ({edge_A_up}_1): tắc nghẽn")
                print(f"   └─ {msg_A}")

            if incident_B:
                print(f" [ALERT] CHIỀU B BỊ TẮC!")
                print(f"   ├─ Làn B-0 ({edge_B_up}_0): tắc nghẽn")
                print(f"   ├─ Làn B-1 ({edge_B_up}_1): tắc nghẽn")
                print(f"   └─ {msg_B}")

            data.append({
                'time'        : t,
                'speed_A_up'  : speed_A_up,
                'speed_A_down': speed_A_down,
                'occ_A_up'    : occ_A_up,
                'occ_A_down'  : occ_A_down,
                'incident_A'  : incident_A,
                'speed_B_up'  : speed_B_up,
                'speed_B_down': speed_B_down,
                'occ_B_up'    : occ_B_up,
                'occ_B_down'  : occ_B_down,
                'incident_B'  : incident_B,
            })

    except traci.FatalTraCIError:
        print(" SUMO bị đóng đột ngột")
    except Exception as e:
        print(f" Lỗi: {e}")
    finally:
        traci.close()
        df = pd.DataFrame(data)
        df.to_csv('D:/Code/ITS_Project/data/final_detection_log.csv', index=False)

        print(f"\n KẾT QUẢ:")
        print(f"   Chiều bị tai nạn:      Chiều {incident_direction}")
        print(f"   Tai nạn xảy ra lúc:    t={incident_time}s")
        print(f"   Cảnh báo chiều A:      {df['incident_A'].sum()}")
        print(f"   Cảnh báo chiều B:      {df['incident_B'].sum()}")
        print(f" Đã lưu final_detection_log.csv!")

if __name__ == "__main__":
    run_simulation()