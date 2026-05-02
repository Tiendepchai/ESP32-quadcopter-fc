#!/usr/bin/env python3
import socket
import time
from dataclasses import dataclass

UDP_PORT = 14550
DISCOVERY_PORT = 14555
BOARD_NAME = "FC_ESP32_QUAD"
WATCHDOG_TIMEOUT_S = 0.8

@dataclass
class PID:
    kp: float
    ki: float
    kd: float


def clamp_pid(pid: PID) -> PID:
    return PID(max(0.01, min(pid.kp, 0.8)), max(0.00, min(pid.ki, 0.30)), max(0.00, min(pid.kd, 0.08)))


def discover_board(timeout_s=5.0):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", DISCOVERY_PORT))
    s.settimeout(0.5)
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        try:
            data, addr = s.recvfrom(512)
        except socket.timeout:
            continue
        msg = data.decode(errors="ignore").strip()
        if msg.startswith("DISC,") and BOARD_NAME in msg:
            s.close()
            return addr[0]
    s.close()
    return "255.255.255.255"


def send_pid(sock, target_ip, pid):
    sock.sendto(f"PID,{pid.kp:.5f},{pid.ki:.5f},{pid.kd:.5f}\n".encode(), (target_ip, UDP_PORT))


def parse_attx(line: str):
    p = line.strip().split(",")
    # ATTX,seq,ms,roll,pitch,yaw,kp,ki,kd,thr,arm,mode
    if len(p) < 12 or p[0] != "ATTX":
        return None
    return {
        "seq": int(p[1]), "ms": int(p[2]),
        "roll": float(p[3]), "pitch": float(p[4]), "yaw": float(p[5]),
        "kp": float(p[6]), "ki": float(p[7]), "kd": float(p[8]),
        "thr": float(p[9]), "arm": int(p[10]), "mode": int(p[11]),
    }


def score_window(samples, packet_loss):
    if not samples:
        return 1e9
    abs_roll = sum(abs(s["roll"]) for s in samples) / len(samples)
    abs_pitch = sum(abs(s["pitch"]) for s in samples) / len(samples)
    disarm_penalty = 200.0 if any(s["arm"] == 0 for s in samples) else 0.0
    high_angle_penalty = 800.0 if any(abs(s["roll"]) > 35 or abs(s["pitch"]) > 35 for s in samples) else 0.0
    loss_penalty = 100.0 * packet_loss
    return abs_roll + abs_pitch + disarm_penalty + high_angle_penalty + loss_penalty


def evaluate_pid(sock, target_ip, pid, horizon_s=4.0):
    send_pid(sock, target_ip, pid)
    t0 = time.time()
    last_rx = t0
    samples = []
    prev_seq = None
    drops = 0

    while time.time() - t0 < horizon_s:
        if time.time() - last_rx > WATCHDOG_TIMEOUT_S:
            return 1e9
        try:
            data, _ = sock.recvfrom(1024)
        except socket.timeout:
            continue

        msg = data.decode(errors="ignore")
        if msg.startswith("ACK,PID,REJECT"):
            return 1e9

        m = parse_attx(msg)
        if not m:
            continue
        last_rx = time.time()
        if prev_seq is not None and m["seq"] > prev_seq + 1:
            drops += (m["seq"] - prev_seq - 1)
        prev_seq = m["seq"]
        samples.append(m)

    packet_loss = drops / max(1, len(samples) + drops)
    return score_window(samples, packet_loss)


def twiddle(sock, target_ip, pid0: PID, d0=(0.02, 0.01, 0.002), iterations=12):
    p = [pid0.kp, pid0.ki, pid0.kd]
    d = list(d0)
    best = evaluate_pid(sock, target_ip, clamp_pid(PID(*p)))
    print(f"init score={best:.3f} pid={p}")

    for it in range(iterations):
        for i in range(3):
            p[i] += d[i]
            score = evaluate_pid(sock, target_ip, clamp_pid(PID(*p)))
            if score < best:
                best = score
                d[i] *= 1.1
            else:
                p[i] -= 2 * d[i]
                score = evaluate_pid(sock, target_ip, clamp_pid(PID(*p)))
                if score < best:
                    best = score
                    d[i] *= 1.05
                else:
                    p[i] += d[i]
                    d[i] *= 0.9
        print(f"iter={it+1:02d} best={best:.3f} pid={p} d={d}")
    return clamp_pid(PID(*p)), best


def main():
    target_ip = discover_board()
    print(f"target={target_ip}")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(("", UDP_PORT))
    sock.settimeout(0.2)

    init_pid = PID(0.12, 0.04, 0.002)
    best_pid, best_score = twiddle(sock, target_ip, init_pid)
    print(f"BEST: {best_pid} score={best_score:.3f}")


if __name__ == "__main__":
    main()
