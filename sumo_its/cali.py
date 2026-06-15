class CaliforniaAlgorithm:
    def __init__(self, speed_threshold=0.5, occ_diff_threshold=15.0, occ_ratio_threshold=0.5):
        self.speed_threshold = speed_threshold
        self.occ_diff_threshold = occ_diff_threshold
        self.occ_ratio_threshold = occ_ratio_threshold

    def detect(self, occ_up, occ_down, speed_up, speed_down, t):
        if t < 30:
            return False, ""

        occ_up_pct   = occ_up * 100
        occ_down_pct = occ_down * 100

        if speed_up > 0.5:
            speed_ratio = speed_down / speed_up
        else:
            speed_ratio = 0.0  # Upstream gần 0 → coi như ratio = 0 → kích hoạt cond_speed

        occ_diff = occ_up_pct - occ_down_pct

        if occ_up_pct > 0:
            occ_ratio = occ_diff / occ_up_pct
        else:
            occ_ratio = 0.0

        cond_speed = speed_ratio < self.speed_threshold
        cond_occ   = (occ_diff > self.occ_diff_threshold) and (occ_ratio > self.occ_ratio_threshold)

        is_incident = cond_speed or cond_occ

        status = "INCIDENT DETECTED!" if is_incident else "NORMAL"
        msg = (f"t={t:.1f}s | speed_ratio={speed_ratio:.2f} | "
               f"occ_diff={occ_diff:.2f}% | occ_ratio={occ_ratio:.2f} | {status}")

        return is_incident, msg