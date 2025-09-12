# Cooperative Adaptive Cruise Control
class CACC:
    def __init__(self, k_p=0.2, k_v=0.4, k_a=0.2, s0=2.0, T=0.1):
        self.k_p = k_p
        self.k_v = k_v
        self.k_a = k_a
        self.s0 = s0
        self.T = T

    def desired_gap(self, v_self):
        return self.s0 + self.T * v_self

    def compute_accel(self, v_self, s_front, v_front, a_front=0.0):
        s_des = self.desired_gap(v_self)
        gap_error = s_front - s_des
        vel_error = v_front - v_self
        accel = (self.k_p * gap_error +
                 self.k_v * vel_error +
                 self.k_a * a_front)
        return float(accel)
