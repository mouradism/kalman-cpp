"""
Reference implementation prototype for validation, simulation,
and test-vector generation matching C++ production behavior.
ISO 26262 ASIL B & ASPICE SWE.1/SWE.2 Reference Model.
"""
from dataclasses import dataclass, field
from enum import IntEnum
from typing import List, Optional, Tuple
import numpy as np

CYCLE_TIME_SEC = 0.040  # 40 ms (25 Hz)
MAX_TRACKS = 64
MAX_DETECTIONS = 128

class TrackStatus(IntEnum):
    FREE = 0
    TENTATIVE = 1
    CONFIRMED = 2
    COASTED = 3
    DELETED = 4

@dataclass
class Detection:
    range_m: float
    azimuth_rad: float
    doppler_mps: float
    snr_db: float = 15.0
    timestamp_ms: int = 0

@dataclass
class TrackOutput:
    track_id: int
    status: TrackStatus
    x_m: float
    y_m: float
    vx_mps: float
    vy_mps: float
    var_x: float
    var_y: float
    var_vx: float
    var_vy: float
    age_frames: int
    coast_count: int
    is_valid_safety: bool

@dataclass
class TrackerConfig:
    sigma_a: float = 2.5          # Process noise acceleration std [m/s^2]
    sigma_r: float = 0.25         # Range measurement noise std [m]
    sigma_theta: float = 0.0087   # Angle measurement noise std [rad] (~0.5 deg)
    sigma_vr: float = 0.15        # Radial velocity noise std [m/s]
    gate_threshold: float = 12.84 # Chi-Square 3-DOF 99.5% threshold
    confirm_hits: int = 3         # M-of-N confirmation hits
    confirm_window: int = 5       # M-of-N confirmation window
    max_coast_frames: int = 5     # Maximum coasting duration

@dataclass
class TrackState:
    track_id: int
    status: TrackStatus
    x: np.ndarray = field(default_factory=lambda: np.zeros((4, 1), dtype=np.float32))
    P: np.ndarray = field(default_factory=lambda: np.eye(4, dtype=np.float32))
    prev_vx: float = 0.0
    prev_vy: float = 0.0
    hit_count: int = 1
    age: int = 1
    coast_count: int = 0
    total_active_cycles: int = 1
    is_valid_safety: bool = True

def wrap_to_pi(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)

class RadarEKF:
    @staticmethod
    def predict(x: np.ndarray, P: np.ndarray, dt: float, sigma_a: float) -> Tuple[np.ndarray, np.ndarray]:
        F = np.eye(4, dtype=np.float32)
        F[0, 2] = dt
        F[1, 3] = dt

        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        sa2 = sigma_a * sigma_a

        q_pos = sa2 * (dt4 / 4.0)
        q_pos_vel = sa2 * (dt3 / 2.0)
        q_vel = sa2 * dt2

        Q = np.array([
            [q_pos,     0.0,       q_pos_vel, 0.0],
            [0.0,       q_pos,     0.0,       q_pos_vel],
            [q_pos_vel, 0.0,       q_vel,     0.0],
            [0.0,       q_pos_vel, 0.0,       q_vel]
        ], dtype=np.float32)

        x_pred = F @ x
        P_pred = F @ P @ F.T + Q
        P_pred = 0.5 * (P_pred + P_pred.T)
        return x_pred, P_pred

    @staticmethod
    def observation_model(x: np.ndarray) -> np.ndarray:
        px = float(x[0, 0])
        py = float(x[1, 0])
        vx = float(x[2, 0])
        vy = float(x[3, 0])

        rho = max(float(np.sqrt(px * px + py * py)), 0.1)
        theta = float(np.arctan2(py, px))
        vr = float((px * vx + py * vy) / rho)
        return np.array([[rho], [theta], [vr]], dtype=np.float32)

    @staticmethod
    def jacobian(x: np.ndarray) -> np.ndarray:
        px = float(x[0, 0])
        py = float(x[1, 0])
        vx = float(x[2, 0])
        vy = float(x[3, 0])

        rho = max(float(np.sqrt(px * px + py * py)), 0.1)
        rho2 = rho * rho
        rho3 = rho2 * rho

        H = np.zeros((3, 4), dtype=np.float32)
        H[0, 0] = px / rho
        H[0, 1] = py / rho
        H[1, 0] = -py / rho2
        H[1, 1] = px / rho2
        H[2, 0] = (py * (vx * py - vy * px)) / rho3
        H[2, 1] = (px * (vy * px - vx * py)) / rho3
        H[2, 2] = px / rho
        H[2, 3] = py / rho
        return H

    @staticmethod
    def innovation(z: np.ndarray, z_pred: np.ndarray) -> np.ndarray:
        y = z - z_pred
        y[1, 0] = wrap_to_pi(float(y[1, 0]))
        return y

    @staticmethod
    def update_joseph(
        x_pred: np.ndarray,
        P_pred: np.ndarray,
        y: np.ndarray,
        H: np.ndarray,
        R: np.ndarray,
        S_inv: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        K = P_pred @ H.T @ S_inv
        x_up = x_pred + K @ y

        I = np.eye(4, dtype=np.float32)
        I_KH = I - K @ H
        P_up = I_KH @ P_pred @ I_KH.T + K @ R @ K.T
        P_up = 0.5 * (P_up + P_up.T)
        return x_up, P_up

class RadarTracker:
    def __init__(self, config: Optional[TrackerConfig] = None):
        self.config = config or TrackerConfig()
        self.R = np.diag([
            self.config.sigma_r ** 2,
            self.config.sigma_theta ** 2,
            self.config.sigma_vr ** 2
        ]).astype(np.float32)
        self.tracks: List[TrackState] = []
        self.next_track_id = 1

    def is_valid_input(self, det: Detection) -> bool:
        if not (0.2 <= det.range_m <= 200.0):
            return False
        if abs(det.azimuth_rad) > 1.04719755:  # 60 degrees
            return False
        if abs(det.doppler_mps) > 70.0:
            return False
        return True

    def process_cycle(self, detections: List[Detection]) -> List[TrackOutput]:
        valid_dets = [d for d in detections[:MAX_DETECTIONS] if self.is_valid_input(d)]

        # 1. Prediction for active tracks
        active_tracks = [t for t in self.tracks if t.status in (TrackStatus.TENTATIVE, TrackStatus.CONFIRMED, TrackStatus.COASTED)]
        pred_states = []
        for track in active_tracks:
            track.prev_vx = float(track.x[2, 0])
            track.prev_vy = float(track.x[3, 0])
            x_pred, P_pred = RadarEKF.predict(track.x, track.P, CYCLE_TIME_SEC, self.config.sigma_a)
            H = RadarEKF.jacobian(x_pred)
            S = H @ P_pred @ H.T + self.R
            try:
                S_inv = np.linalg.inv(S)
            except np.linalg.LinAlgError:
                S_inv = np.zeros((3, 3), dtype=np.float32)
            z_pred = RadarEKF.observation_model(x_pred)
            pred_states.append((x_pred, P_pred, H, S_inv, z_pred))

        # 2. Spatial gating & cost matrix
        cost_matrix = np.full((len(active_tracks), len(valid_dets)), 1e8, dtype=np.float32)
        for i, (track, (x_pred, P_pred, H, S_inv, z_pred)) in enumerate(zip(active_tracks, pred_states)):
            for j, det in enumerate(valid_dets):
                z_meas = np.array([[det.range_m], [det.azimuth_rad], [det.doppler_mps]], dtype=np.float32)
                y = RadarEKF.innovation(z_meas, z_pred)
                d2 = float(y.T @ S_inv @ y)
                if 0.0 <= d2 <= self.config.gate_threshold:
                    cost_matrix[i, j] = d2

        # 3. Association (Hungarian matching or greedy GNN)
        assigned_tracks = {}
        assigned_dets = set()
        if len(active_tracks) > 0 and len(valid_dets) > 0:
            from scipy.optimize import linear_sum_assignment
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
            for r, c in zip(row_ind, col_ind):
                if cost_matrix[r, c] <= self.config.gate_threshold:
                    assigned_tracks[r] = c
                    assigned_dets.add(c)

        # 4. Updates & Lifecycle transitions
        for i, (track, (x_pred, P_pred, H, S_inv, z_pred)) in enumerate(zip(active_tracks, pred_states)):
            if i in assigned_tracks:
                det = valid_dets[assigned_tracks[i]]
                z_meas = np.array([[det.range_m], [det.azimuth_rad], [det.doppler_mps]], dtype=np.float32)
                y = RadarEKF.innovation(z_meas, z_pred)
                x_up, P_up = RadarEKF.update_joseph(x_pred, P_pred, y, H, self.R, S_inv)

                track.x = x_up
                track.P = P_up
                track.hit_count += 1
                track.age += 1
                track.total_active_cycles += 1

                if track.status == TrackStatus.TENTATIVE:
                    if track.hit_count >= self.config.confirm_hits and track.age <= self.config.confirm_window:
                        track.status = TrackStatus.CONFIRMED
                else:
                    track.status = TrackStatus.CONFIRMED
                    track.coast_count = 0
            else:
                track.x = x_pred
                track.P = P_pred
                track.age += 1
                track.total_active_cycles += 1

                if track.status == TrackStatus.TENTATIVE:
                    misses = track.age - track.hit_count
                    if track.age >= self.config.confirm_window and track.hit_count < self.config.confirm_hits:
                        track.status = TrackStatus.DELETED
                    elif misses > (self.config.confirm_window - self.config.confirm_hits):
                        track.status = TrackStatus.DELETED
                elif track.status == TrackStatus.CONFIRMED:
                    track.status = TrackStatus.COASTED
                    track.coast_count = 1
                elif track.status == TrackStatus.COASTED:
                    track.coast_count += 1
                    if track.coast_count > self.config.max_coast_frames:
                        track.status = TrackStatus.DELETED

        # 5. Initialize unassigned detections
        for j, det in enumerate(valid_dets):
            if j not in assigned_dets and len(self.tracks) < MAX_TRACKS:
                r, th, vr = det.range_m, det.azimuth_rad, det.doppler_mps
                x0 = np.array([
                    [r * np.cos(th)],
                    [r * np.sin(th)],
                    [vr * np.cos(th)],
                    [vr * np.sin(th)]
                ], dtype=np.float32)

                var_r = self.config.sigma_r ** 2
                lat_var = (r * self.config.sigma_theta) ** 2
                P0 = np.diag([max(var_r, 1.0), max(lat_var, 1.0), 25.0, 25.0]).astype(np.float32)

                new_track = TrackState(
                    track_id=self.next_track_id,
                    status=TrackStatus.TENTATIVE,
                    x=x0,
                    P=P0,
                    prev_vx=float(x0[2, 0]),
                    prev_vy=float(x0[3, 0]),
                    hit_count=1,
                    age=1,
                    coast_count=0,
                    total_active_cycles=1,
                    is_valid_safety=True
                )
                self.next_track_id += 1
                self.tracks.append(new_track)

        # 6. Safety & Cleanup
        surviving_tracks = []
        for track in self.tracks:
            if track.status in (TrackStatus.TENTATIVE, TrackStatus.CONFIRMED, TrackStatus.COASTED):
                vx = float(track.x[2, 0])
                vy = float(track.x[3, 0])
                speed = float(np.sqrt(vx * vx + vy * vy))
                if speed > 85.0:
                    track.is_valid_safety = False

                if track.age > 1:
                    ax = (vx - track.prev_vx) / CYCLE_TIME_SEC
                    ay = (vy - track.prev_vy) / CYCLE_TIME_SEC
                    accel = float(np.sqrt(ax * ax + ay * ay))
                    if accel > 15.0:
                        track.is_valid_safety = False

                if np.trace(track.P) > 1000.0 or not np.isfinite(np.trace(track.P)):
                    track.status = TrackStatus.DELETED

                px, py = float(track.x[0, 0]), float(track.x[1, 0])
                if px < 0.2 or px > 250.0 or abs(py) > (px * 1.8 + 15.0):
                    track.status = TrackStatus.DELETED

            if track.status != TrackStatus.DELETED:
                surviving_tracks.append(track)
        self.tracks = surviving_tracks

        # 7. Output list
        outputs = []
        for track in self.tracks:
            if track.status in (TrackStatus.CONFIRMED, TrackStatus.COASTED):
                speed = float(np.sqrt(track.x[2, 0] ** 2 + track.x[3, 0] ** 2))
                if speed < 0.5 and (track.total_active_cycles * CYCLE_TIME_SEC) < 0.150:
                    continue
                outputs.append(TrackOutput(
                    track_id=track.track_id,
                    status=track.status,
                    x_m=float(track.x[0, 0]),
                    y_m=float(track.x[1, 0]),
                    vx_mps=float(track.x[2, 0]),
                    vy_mps=float(track.x[3, 0]),
                    var_x=float(track.P[0, 0]),
                    var_y=float(track.P[1, 1]),
                    var_vx=float(track.P[2, 2]),
                    var_vy=float(track.P[3, 3]),
                    age_frames=track.age,
                    coast_count=track.coast_count,
                    is_valid_safety=track.is_valid_safety
                ))
        return outputs

if __name__ == "__main__":
    tracker = RadarTracker()
    print("[Python Model Initialized] Target cycle time:", CYCLE_TIME_SEC * 1000, "ms")
    sample_det = Detection(range_m=50.0, azimuth_rad=0.02, doppler_mps=-12.5)
    out = tracker.process_cycle([sample_det])
    print(f"Frame 1: {len(out)} confirmed tracks, {len(tracker.tracks)} internal tracks")
