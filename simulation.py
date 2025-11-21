"""
Mixed traffic platoon simulation harness.

This script wires the existing controllers (ACC, CACC, IDM) into a simple
time-step simulation so you can exercise mixed CAV/HDV platoons, add
disturbances (noise, delay), and collect metrics like spacing, speed, and TTC.
"""

from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from Controller.ACC import ACC
from Controller.CACC import CACC
from Controller.IDM import IDMController


def safe_gap(lead_x: float, self_x: float, vehicle_length: float) -> float:
    """Compute rear-to-front bumper gap."""
    return max(lead_x - self_x - vehicle_length, 1e-3)


def time_to_collision(gap: float, rel_speed: float) -> Optional[float]:
    """
    TTC based on closing speed (positive rel_speed means follower is faster).
    Returns None if not closing in.
    """
    if rel_speed <= 0:
        return None
    return gap / rel_speed


@dataclass
class Vehicle:
    vehicle_id: int
    kind: str  # "CAV" or "HDV"
    controller: object
    x: float
    v: float
    a: float = 0.0
    vehicle_length: float = 5.0
    accel_noise_std: float = 0.0
    delay_steps: int = 0
    # Internal buffers for delayed actuation/measurement
    _accel_buffer: deque = field(init=False)

    def __post_init__(self):
        self._accel_buffer = deque([0.0] * (self.delay_steps + 1), maxlen=self.delay_steps + 1)

    def push_accel(self, a_cmd: float):
        """Apply delay by pushing through a buffer."""
        self._accel_buffer.append(a_cmd)
        self.a = self._accel_buffer[0]  # oldest value exits buffer

    def step(self, dt: float):
        """Integrate simple point-mass longitudinal dynamics."""
        self.v = max(self.v + self.a * dt, 0.0)
        self.x += self.v * dt


class PlatoonSimulator:
    def __init__(self, vehicles: List[Vehicle], dt: float = 0.1):
        self.vehicles = vehicles  # sorted front to back
        self.dt = dt
        self.history: Dict[str, List] = {k: [] for k in ["t", "x", "v", "a", "gap", "ttc"]}

    def lead_profile(self, t: float) -> float:
        """
        Simple scripted leader acceleration profile.
        - cruise at 20 m/s
        - hard brake between 10s-12s: -2 m/s^2
        """
        if 10.0 <= t <= 12.0:
            return -2.0
        return 0.0

    def compute_accels(self, t: float):
        # Leader (index 0) follows profile
        lead = self.vehicles[0]
        a_lead = self.lead_profile(t)
        lead.push_accel(a_lead)

        # Followers
        for idx in range(1, len(self.vehicles)):
            veh = self.vehicles[idx]
            lead = self.vehicles[idx - 1]
            gap = safe_gap(lead_x=lead.x, self_x=veh.x, vehicle_length=veh.vehicle_length)
            rel_speed = lead.v - veh.v
            ttc = time_to_collision(gap, -rel_speed)  # closing if veh faster -> rel_speed negative

            # Base accel command from controller
            if hasattr(veh.controller, "compute_accel"):
                # ACC / CACC
                if isinstance(veh.controller, CACC):
                    a_cmd = veh.controller.compute_accel(
                        v_self=veh.v, s_front=gap, v_front=lead.v, a_front=lead.a, ttc=ttc
                    )
                else:
                    a_cmd = veh.controller.compute_accel(
                        v_self=veh.v, s_front=gap, v_front=lead.v, ttc=ttc
                    )
            elif isinstance(veh.controller, IDMController):
                delta_v = veh.v - lead.v  # follower minus leader
                a_cmd = veh.controller.calculate_acceleration(v=veh.v, s=gap, delta_v=delta_v)
            else:
                raise ValueError(f"Unknown controller type for vehicle {veh.vehicle_id}")

            # Disturbance: add Gaussian noise
            if veh.accel_noise_std > 0:
                a_cmd += np.random.normal(0.0, veh.accel_noise_std)

            # Apply buffering for actuation delay and update
            veh.push_accel(a_cmd)

    def log_state(self, t: float):
        xs = [v.x for v in self.vehicles]
        vs = [v.v for v in self.vehicles]
        accs = [v.a for v in self.vehicles]
        gaps = [np.nan]  # leader has no gap
        ttcs = [None]
        for idx in range(1, len(self.vehicles)):
            lead = self.vehicles[idx - 1]
            veh = self.vehicles[idx]
            gap = safe_gap(lead.x, veh.x, veh.vehicle_length)
            rel_speed = veh.v - lead.v
            gaps.append(gap)
            ttcs.append(time_to_collision(gap, rel_speed))
        self.history["t"].append(t)
        self.history["x"].append(xs)
        self.history["v"].append(vs)
        self.history["a"].append(accs)
        self.history["gap"].append(gaps)
        self.history["ttc"].append(ttcs)

    def step(self, t: float):
        self.compute_accels(t)
        for v in self.vehicles:
            v.step(self.dt)
        self.log_state(t)

    def run(self, T: float):
        steps = int(T / self.dt)
        for k in self.history:
            self.history[k] = []  # reset
        t = 0.0
        for _ in range(steps):
            self.step(t)
            t += self.dt
        return self.history


def build_demo_platoon(
    dt: float = 0.1, noise_std: float = 0.2, k_ttc: float = 2.0, min_ttc: float = 0.8
) -> PlatoonSimulator:
    """
    Build a 10-vehicle platoon:
    - Vehicle 0: leader, scripted brake event
    - Vehicles 1-3: CAVs using CACC with TTC term (set in controller)
    - Vehicles 4-9: HDVs using IDM, with accel noise
    """
    vehicles: List[Vehicle] = []

    # Leader with simple placeholder controller (not used)
    leader = Vehicle(vehicle_id=0, kind="Leader", controller=ACC(), x=0.0, v=20.0)
    vehicles.append(leader)

    # CAVs
    for i in range(1, 4):
        controller = CACC(
            k_p=0.3,
            k_v=0.6,
            k_a=0.3,
            s0=5.0,
            T=0.7,
            k_ttc=k_ttc,
            min_ttc=min_ttc,
        )
        vehicles.append(
            Vehicle(
                vehicle_id=i,
                kind="CAV",
                controller=controller,
                x=-i * 10.0,
                v=20.0,
                delay_steps=1,
            )
        )

    # HDVs
    for i in range(4, 10):
        controller = IDMController(v0=25.0, T=1.2, a=1.2, b=1.8, s0=2.0, delta=4)
        vehicles.append(
            Vehicle(
                vehicle_id=i,
                kind="HDV",
                controller=controller,
                x=-i * 10.0,
                v=20.0,
                accel_noise_std=noise_std,
                delay_steps=2,
            )
        )

    return PlatoonSimulator(vehicles=vehicles, dt=dt)


def summarize_metrics(history: Dict[str, List]) -> Dict[str, float]:
    """Compute basic metrics for quick comparison."""
    gaps = np.array(history["gap"])[:, 1:]  # ignore leader None
    ttcs = np.array([[t if t is not None else np.nan for t in row[1:]] for row in history["ttc"]])
    v = np.array(history["v"])
    a = np.array(history["a"])

    # Safety
    min_ttc = np.nanmin(ttcs)
    min_gap = np.nanmin(gaps)

    # Comfort/energy proxy
    if len(history["t"]) > 1:
        dt = history["t"][1] - history["t"][0]
        jerk = np.diff(a, axis=0) / dt
    else:
        jerk = np.array([])
    max_accel = np.nanmax(a)
    max_decel = np.nanmin(a)

    # String stability proxy: std of speed across platoon
    speed_std = float(np.mean(np.std(v, axis=1)))

    return {
        "min_ttc": float(min_ttc),
        "min_gap": float(min_gap),
        "max_accel": float(max_accel),
        "max_decel": float(max_decel),
        "speed_std": speed_std,
        "jerk_mean_abs": float(np.mean(np.abs(jerk))) if jerk.size else 0.0,
    }


if __name__ == "__main__":
    sim = build_demo_platoon()
    history = sim.run(T=30.0)
    metrics = summarize_metrics(history)
    print("Demo run metrics:", metrics)
