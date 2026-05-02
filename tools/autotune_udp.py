#!/usr/bin/env python3
import argparse
import json
import socket
import sys
import time
from dataclasses import asdict, dataclass
from typing import Callable, List, Optional, Tuple

UDP_PORT = 14550
DISCOVERY_PORT = 14555
BOARD_NAME = "FC_ESP32_QUAD"
WATCHDOG_TIMEOUT_S = 0.8
INF_SCORE = 1e9


@dataclass
class AxisPid:
    kp: float
    ki: float
    kd: float


@dataclass
class AttxSample:
    seq: int
    ms: int
    roll: float
    pitch: float
    yaw: float
    roll_rate: float
    pitch_rate: float
    yaw_rate: float
    roll_sp: float
    pitch_sp: float
    yaw_sp: float
    rp_kp: float
    rp_ki: float
    rp_kd: float
    y_kp: float
    y_ki: float
    y_kd: float
    thr: float
    arm: int
    mode: int
    tune_ok: int


@dataclass
class Ack:
    cmd: str
    ok: bool
    reason: str = ""


@dataclass
class WindowResult:
    samples: List[AttxSample]
    packet_loss: float
    watchdog_triggered: bool


@dataclass
class ScoreBreakdown:
    score: float
    hold_metric: float
    tracking_metric: float
    oscillation_metric: float
    coupling_metric: float
    packet_loss: float
    center_samples: int
    maneuver_samples: int
    penalties: float
    note: str


def clamp_axis_pid(pid: AxisPid) -> AxisPid:
    return AxisPid(
        max(0.01, min(pid.kp, 0.80)),
        max(0.00, min(pid.ki, 0.30)),
        max(0.00, min(pid.kd, 0.08)),
    )


def mean(values: List[float], default: float = 0.0) -> float:
    return sum(values) / len(values) if values else default


def mean_abs_delta(values: List[float]) -> float:
    if len(values) < 2:
        return 0.0
    return sum(abs(values[i] - values[i - 1]) for i in range(1, len(values))) / (len(values) - 1)


def discover_board(timeout_s: float = 5.0, board_name: str = BOARD_NAME) -> str:
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
        if msg.startswith("DISC,") and board_name in msg:
            s.close()
            return addr[0]
    s.close()
    return "255.255.255.255"


def parse_ack(line: str) -> Optional[Ack]:
    p = line.strip().split(",")
    if len(p) < 3 or p[0] != "ACK":
        return None
    return Ack(cmd=p[1], ok=(p[2] == "OK"), reason=p[3] if len(p) > 3 else "")


def parse_attx(line: str) -> Optional[AttxSample]:
    p = line.strip().split(",")
    if not p or p[0] != "ATTX":
        return None

    if len(p) >= 22:
        return AttxSample(
            seq=int(p[1]),
            ms=int(p[2]),
            roll=float(p[3]),
            pitch=float(p[4]),
            yaw=float(p[5]),
            roll_rate=float(p[6]),
            pitch_rate=float(p[7]),
            yaw_rate=float(p[8]),
            roll_sp=float(p[9]),
            pitch_sp=float(p[10]),
            yaw_sp=float(p[11]),
            rp_kp=float(p[12]),
            rp_ki=float(p[13]),
            rp_kd=float(p[14]),
            y_kp=float(p[15]),
            y_ki=float(p[16]),
            y_kd=float(p[17]),
            thr=float(p[18]),
            arm=int(p[19]),
            mode=int(p[20]),
            tune_ok=int(p[21]),
        )

    if len(p) >= 12:
        return AttxSample(
            seq=int(p[1]),
            ms=int(p[2]),
            roll=float(p[3]),
            pitch=float(p[4]),
            yaw=float(p[5]),
            roll_rate=0.0,
            pitch_rate=0.0,
            yaw_rate=0.0,
            roll_sp=0.0,
            pitch_sp=0.0,
            yaw_sp=0.0,
            rp_kp=float(p[6]),
            rp_ki=float(p[7]),
            rp_kd=float(p[8]),
            y_kp=float(p[6]),
            y_ki=float(p[7]),
            y_kd=float(p[8]),
            thr=float(p[9]),
            arm=int(p[10]),
            mode=int(p[11]),
            tune_ok=1 if int(p[10]) == 0 else 0,
        )
    return None


class UdpTuneLink:
    def __init__(self, target_ip: str, port: int = UDP_PORT):
        self.target_ip = target_ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(("", port))
        self.sock.settimeout(0.2)
        self.last_sample: Optional[AttxSample] = None

    def close(self) -> None:
        self.sock.close()

    def recv_message(self, timeout_s: float = 0.5):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            try:
                data, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            msg = data.decode(errors="ignore").strip()
            ack = parse_ack(msg)
            if ack:
                return "ack", ack
            sample = parse_attx(msg)
            if sample:
                self.last_sample = sample
                return "attx", sample
        return None, None

    def drain(self, duration_s: float = 0.25) -> None:
        deadline = time.time() + duration_s
        while time.time() < deadline:
            kind, _ = self.recv_message(timeout_s=min(0.1, deadline - time.time()))
            if kind is None:
                break

    def wait_for_telemetry(self, timeout_s: float = 5.0) -> AttxSample:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            kind, msg = self.recv_message(timeout_s=0.5)
            if kind == "attx":
                return msg
        raise TimeoutError("telemetry timeout")

    def wait_for_tune_window(self, timeout_s: float = 15.0, consecutive: int = 5) -> AttxSample:
        deadline = time.time() + timeout_s
        ready = 0
        while time.time() < deadline:
            sample = self.wait_for_telemetry(timeout_s=min(1.0, deadline - time.time()))
            if sample.arm == 0 or sample.tune_ok:
                ready += 1
                if ready >= consecutive:
                    return sample
            else:
                ready = 0
        raise TimeoutError("no safe tune window")

    def send_pid(self, cmd: str, pid: AxisPid) -> None:
        pid = clamp_axis_pid(pid)
        payload = f"{cmd},{pid.kp:.5f},{pid.ki:.5f},{pid.kd:.5f}\n"
        self.sock.sendto(payload.encode(), (self.target_ip, self.port))

    def apply_pid(self, cmd: str, pid: AxisPid, ready_timeout_s: float = 15.0, retries: int = 4) -> AxisPid:
        pid = clamp_axis_pid(pid)
        last_reason = "timeout"
        for _ in range(retries):
            self.wait_for_tune_window(timeout_s=ready_timeout_s)
            self.drain(0.1)
            self.send_pid(cmd, pid)
            ack_deadline = time.time() + 2.0
            while time.time() < ack_deadline:
                kind, msg = self.recv_message(timeout_s=0.4)
                if kind == "ack" and msg.cmd == cmd:
                    if msg.ok:
                        return pid
                    last_reason = msg.reason or "rejected"
                    if msg.reason in {"COOLDOWN", "STICKS", "THROTTLE", "MODE", "ARMED_DISABLED"}:
                        break
                    raise RuntimeError(f"{cmd} rejected: {last_reason}")
        raise RuntimeError(f"{cmd} apply failed: {last_reason}")

    def collect_window(self, duration_s: float, watchdog_s: float) -> WindowResult:
        deadline = time.time() + duration_s
        last_rx = time.time()
        prev_seq = None
        drops = 0
        samples: List[AttxSample] = []

        while time.time() < deadline:
            if time.time() - last_rx > watchdog_s:
                return WindowResult(samples=samples, packet_loss=1.0, watchdog_triggered=True)
            kind, msg = self.recv_message(timeout_s=min(0.25, deadline - time.time()))
            if kind == "attx":
                last_rx = time.time()
                if prev_seq is not None and msg.seq > prev_seq + 1:
                    drops += msg.seq - prev_seq - 1
                prev_seq = msg.seq
                samples.append(msg)

        packet_loss = drops / max(1, len(samples) + drops)
        return WindowResult(samples=samples, packet_loss=packet_loss, watchdog_triggered=False)


def score_roll_pitch(window: WindowResult, hover_throttle_min: float) -> ScoreBreakdown:
    if window.watchdog_triggered or not window.samples:
        return ScoreBreakdown(INF_SCORE, 0.0, 0.0, 0.0, 0.0, window.packet_loss, 0, 0, 0.0, "watchdog")

    armed = [s for s in window.samples if s.arm == 1 and s.thr >= hover_throttle_min]
    if len(armed) < 20:
        return ScoreBreakdown(INF_SCORE, 0.0, 0.0, 0.0, 0.0, window.packet_loss, 0, 0, 0.0, "not_enough_armed_samples")

    center = [s for s in armed if abs(s.roll_sp) <= 12.0 and abs(s.pitch_sp) <= 12.0]
    maneuver = [s for s in armed if abs(s.roll_sp) >= 25.0 or abs(s.pitch_sp) >= 25.0]

    hold_angle = mean([(abs(s.roll) + abs(s.pitch)) * 0.5 for s in center], default=45.0)
    tracking = mean(
        [(abs(s.roll_rate - s.roll_sp) + abs(s.pitch_rate - s.pitch_sp)) * 0.5 for s in armed],
        default=250.0,
    )
    oscillation = (
        mean_abs_delta([s.roll_rate for s in center]) +
        mean_abs_delta([s.pitch_rate for s in center])
    ) * 0.5
    center_rate = mean([(abs(s.roll_rate) + abs(s.pitch_rate)) * 0.5 for s in center], default=120.0)

    penalties = 0.0
    if any(abs(s.roll) > 45.0 or abs(s.pitch) > 45.0 for s in armed):
        penalties += 300.0
    if any(s.arm == 0 for s in window.samples):
        penalties += 400.0
    penalties += 150.0 * window.packet_loss
    if len(center) < 12:
        penalties += 120.0
    if len(maneuver) < 10:
        penalties += 90.0

    score = 3.0 * hold_angle + 0.06 * tracking + 0.4 * center_rate + 0.03 * oscillation + penalties
    return ScoreBreakdown(
        score=score,
        hold_metric=hold_angle,
        tracking_metric=tracking,
        oscillation_metric=oscillation,
        coupling_metric=center_rate,
        packet_loss=window.packet_loss,
        center_samples=len(center),
        maneuver_samples=len(maneuver),
        penalties=penalties,
        note="rp",
    )


def score_yaw(window: WindowResult, hover_throttle_min: float) -> ScoreBreakdown:
    if window.watchdog_triggered or not window.samples:
        return ScoreBreakdown(INF_SCORE, 0.0, 0.0, 0.0, 0.0, window.packet_loss, 0, 0, 0.0, "watchdog")

    armed = [s for s in window.samples if s.arm == 1 and s.thr >= hover_throttle_min]
    if len(armed) < 20:
        return ScoreBreakdown(INF_SCORE, 0.0, 0.0, 0.0, 0.0, window.packet_loss, 0, 0, 0.0, "not_enough_armed_samples")

    center = [s for s in armed if abs(s.yaw_sp) <= 18.0]
    maneuver = [s for s in armed if abs(s.yaw_sp) >= 30.0]

    hold_rate = mean([abs(s.yaw_rate) for s in center], default=180.0)
    tracking = mean([abs(s.yaw_rate - s.yaw_sp) for s in armed], default=250.0)
    oscillation = mean_abs_delta([s.yaw_rate for s in center])
    coupling = mean([(abs(s.roll) + abs(s.pitch)) * 0.5 for s in armed], default=45.0)

    penalties = 0.0
    if any(abs(s.roll) > 45.0 or abs(s.pitch) > 45.0 for s in armed):
        penalties += 300.0
    if any(s.arm == 0 for s in window.samples):
        penalties += 400.0
    penalties += 150.0 * window.packet_loss
    if len(center) < 12:
        penalties += 120.0
    if len(maneuver) < 8:
        penalties += 80.0

    score = 1.5 * hold_rate + 0.08 * tracking + 0.05 * oscillation + 1.5 * coupling + penalties
    return ScoreBreakdown(
        score=score,
        hold_metric=hold_rate,
        tracking_metric=tracking,
        oscillation_metric=oscillation,
        coupling_metric=coupling,
        packet_loss=window.packet_loss,
        center_samples=len(center),
        maneuver_samples=len(maneuver),
        penalties=penalties,
        note="yaw",
    )


def print_breakdown(prefix: str, breakdown: ScoreBreakdown, pid: AxisPid) -> None:
    print(
        f"{prefix} score={breakdown.score:.2f} "
        f"pid=({pid.kp:.4f},{pid.ki:.4f},{pid.kd:.4f}) "
        f"hold={breakdown.hold_metric:.2f} "
        f"track={breakdown.tracking_metric:.2f} "
        f"osc={breakdown.oscillation_metric:.2f} "
        f"coupling={breakdown.coupling_metric:.2f} "
        f"loss={breakdown.packet_loss:.3f} "
        f"center={breakdown.center_samples} "
        f"maneuver={breakdown.maneuver_samples} "
        f"pen={breakdown.penalties:.1f}"
    )


def settle_and_prompt(link: UdpTuneLink, settle_s: float, prep_s: float, prompt: str, watchdog_s: float) -> None:
    if settle_s > 0.0:
        print(f"settle {settle_s:.1f}s")
        link.collect_window(settle_s, watchdog_s=watchdog_s)
    if prep_s > 0.0:
        print(prompt)
        deadline = time.time() + prep_s
        while time.time() < deadline:
            link.recv_message(timeout_s=min(0.2, deadline - time.time()))


def evaluate_axis(
    link: UdpTuneLink,
    cmd: str,
    pid: AxisPid,
    settle_s: float,
    prep_s: float,
    horizon_s: float,
    hover_throttle_min: float,
    watchdog_s: float,
    prompt: str,
    score_fn: Callable[[WindowResult, float], ScoreBreakdown],
) -> Tuple[AxisPid, ScoreBreakdown]:
    applied = link.apply_pid(cmd, pid)
    settle_and_prompt(link, settle_s, prep_s, prompt, watchdog_s)
    window = link.collect_window(horizon_s, watchdog_s=watchdog_s)
    breakdown = score_fn(window, hover_throttle_min)
    return applied, breakdown


def twiddle_axis(
    name: str,
    initial_pid: AxisPid,
    delta0: Tuple[float, float, float],
    iterations: int,
    evaluate: Callable[[AxisPid], Tuple[AxisPid, ScoreBreakdown]],
) -> Tuple[AxisPid, ScoreBreakdown]:
    p = [initial_pid.kp, initial_pid.ki, initial_pid.kd]
    d = list(delta0)

    best_pid, best_breakdown = evaluate(clamp_axis_pid(AxisPid(*p)))
    print_breakdown(f"{name} init", best_breakdown, best_pid)

    for it in range(iterations):
        for i in range(3):
            p[i] += d[i]
            cand_pid, cand_breakdown = evaluate(clamp_axis_pid(AxisPid(*p)))
            if cand_breakdown.score < best_breakdown.score:
                best_pid, best_breakdown = cand_pid, cand_breakdown
                p = [best_pid.kp, best_pid.ki, best_pid.kd]
                d[i] *= 1.08
                continue

            p[i] -= 2.0 * d[i]
            cand_pid, cand_breakdown = evaluate(clamp_axis_pid(AxisPid(*p)))
            if cand_breakdown.score < best_breakdown.score:
                best_pid, best_breakdown = cand_pid, cand_breakdown
                p = [best_pid.kp, best_pid.ki, best_pid.kd]
                d[i] *= 1.04
            else:
                p[i] += d[i]
                p = [best_pid.kp, best_pid.ki, best_pid.kd]
                d[i] *= 0.85

        print_breakdown(f"{name} iter={it + 1:02d}", best_breakdown, best_pid)
    return best_pid, best_breakdown


def tuple_to_pid(values: Optional[List[float]], fallback: AxisPid) -> AxisPid:
    if values is None:
        return fallback
    return clamp_axis_pid(AxisPid(values[0], values[1], values[2]))


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Semi-auto UDP PID tuner for FC_ESP32_QUAD")
    p.add_argument("--target-ip", default=None, help="Board IP. Default: auto-discovery")
    p.add_argument("--board-name", default=BOARD_NAME, help="Discovery board name")
    p.add_argument("--rp-init", nargs=3, type=float, default=None, metavar=("KP", "KI", "KD"))
    p.add_argument("--yaw-init", nargs=3, type=float, default=None, metavar=("KP", "KI", "KD"))
    p.add_argument("--rp-step", nargs=3, type=float, default=(0.012, 0.006, 0.0015), metavar=("KP", "KI", "KD"))
    p.add_argument("--yaw-step", nargs=3, type=float, default=(0.010, 0.004, 0.0010), metavar=("KP", "KI", "KD"))
    p.add_argument("--rp-iters", type=int, default=5)
    p.add_argument("--yaw-iters", type=int, default=4)
    p.add_argument("--horizon", type=float, default=4.0, help="Scoring window in seconds")
    p.add_argument("--settle", type=float, default=1.5, help="Wait after each PID change")
    p.add_argument("--prep", type=float, default=2.0, help="Pilot prep time before scoring window")
    p.add_argument("--hover-throttle-min", type=float, default=0.18)
    p.add_argument("--watchdog", type=float, default=WATCHDOG_TIMEOUT_S)
    p.add_argument("--skip-rp", action="store_true")
    p.add_argument("--skip-yaw", action="store_true")
    p.add_argument("--save-best", default=None, help="Optional JSON output path")
    return p


def main() -> int:
    args = build_arg_parser().parse_args()

    target_ip = args.target_ip or discover_board(board_name=args.board_name)
    print(f"target={target_ip}")

    link = UdpTuneLink(target_ip)
    try:
        sample = link.wait_for_telemetry(timeout_s=6.0)
        print(
            f"telemetry mode={sample.mode} arm={sample.arm} tune_ok={sample.tune_ok} "
            f"rp=({sample.rp_kp:.4f},{sample.rp_ki:.4f},{sample.rp_kd:.4f}) "
            f"yaw=({sample.y_kp:.4f},{sample.y_ki:.4f},{sample.y_kd:.4f})"
        )

        rp_init = tuple_to_pid(args.rp_init, AxisPid(sample.rp_kp, sample.rp_ki, sample.rp_kd))
        yaw_init = tuple_to_pid(args.yaw_init, AxisPid(sample.y_kp, sample.y_ki, sample.y_kd))

        best_rp = rp_init
        best_rp_breakdown = ScoreBreakdown(INF_SCORE, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0.0, "skipped")
        best_yaw = yaw_init
        best_yaw_breakdown = ScoreBreakdown(INF_SCORE, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0.0, "skipped")

        if not args.skip_rp:
            def eval_rp(pid: AxisPid):
                return evaluate_axis(
                    link=link,
                    cmd="PIDRP",
                    pid=pid,
                    settle_s=args.settle,
                    prep_s=args.prep,
                    horizon_s=args.horizon,
                    hover_throttle_min=args.hover_throttle_min,
                    watchdog_s=args.watchdog,
                    prompt="RP window: giữ hover ANGLE và thực hiện 2-3 stick bump nhỏ ở roll/pitch rồi về center.",
                    score_fn=score_roll_pitch,
                )

            best_rp, best_rp_breakdown = twiddle_axis(
                "RP",
                rp_init,
                tuple(args.rp_step),
                args.rp_iters,
                eval_rp,
            )

        if not args.skip_yaw:
            link.apply_pid("PIDRP", best_rp)

            def eval_yaw(pid: AxisPid):
                return evaluate_axis(
                    link=link,
                    cmd="PIDY",
                    pid=pid,
                    settle_s=args.settle,
                    prep_s=args.prep,
                    horizon_s=args.horizon,
                    hover_throttle_min=args.hover_throttle_min,
                    watchdog_s=args.watchdog,
                    prompt="Yaw window: giữ hover ổn định, thực hiện 2-3 yaw bump ngắn rồi về center.",
                    score_fn=score_yaw,
                )

            best_yaw, best_yaw_breakdown = twiddle_axis(
                "YAW",
                yaw_init,
                tuple(args.yaw_step),
                args.yaw_iters,
                eval_yaw,
            )

        link.apply_pid("PIDRP", best_rp)
        link.apply_pid("PIDY", best_yaw)

        print_breakdown("RP best", best_rp_breakdown, best_rp)
        print_breakdown("YAW best", best_yaw_breakdown, best_yaw)

        if args.save_best:
            payload = {
                "target_ip": target_ip,
                "timestamp_unix_s": time.time(),
                "roll_pitch_pid": asdict(best_rp),
                "yaw_pid": asdict(best_yaw),
                "roll_pitch_score": asdict(best_rp_breakdown),
                "yaw_score": asdict(best_yaw_breakdown),
            }
            with open(args.save_best, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2)
            print(f"saved={args.save_best}")

        return 0
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    finally:
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
