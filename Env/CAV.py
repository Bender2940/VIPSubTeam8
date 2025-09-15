from acc import ACC
from cacc import CACC


class CAV:

    def __init__(self, x=0.0, v=0.0, length=5.0, F_mode="ACC", acc_params=None, cacc_params=None):
        self.x = x
        self.v = v
        self.a = 0.0
        self.length = length
        self.F_mode = F_mode.upper()

        
        self.acc = ACC(**(acc_params or {}))
        self.cacc = CACC(**(cacc_params or {}))

    def acceleration(self, s_front, v_front, a_front=0.0):
        if self.F_mode == "ACC":
            self.a = self.acc.compute_accel(self.v, s_front, v_front)
        elif self.F_mode == "CACC":
            self.a = self.cacc.compute_accel(self.v, s_front, v_front, a_front)
        else:
            raise ValueError(f"Unknown F_mode: {self.F_mode}")
        return self.a

    def update(self, dt):
        self.v += self.a * dt
        self.v = max(self.v, 0.0)  
        self.x += self.v * dt + 0.5 * self.a * dt**2# Connected and Autonomous Vehicles
