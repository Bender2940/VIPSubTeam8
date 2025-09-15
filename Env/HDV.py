# Human Driven Vehicle
import math as np

class HDV:

    def __init__(self, x=0.0, v=0.0, length=5.0, model="IDM", params=None):
        self.x = x
        self.v = v
        self.a = 0.0
        self.length = length
        self.model = model.upper()
        self.params = params if params else {}
        self._set_default_params()

    def _set_default_params(self):
        if self.model == "IDM":
            defaults = {
                "v0": 30.0,      # desired speed (m/s)
                "T": 2.5,        # desired time headway (s)
                "s0": 2.0,       # minimum gap (m)
                "a_max": 1.0,    # max acceleration (m/s^2)
                "b": 2.0,        # comfortable deceleration (m/s^2)
                "delta": 4       # acceleration exponent
            }
        elif self.model == "OVM":
            defaults = {
                "v_max": 30.0,    # max speed (m/s)
                "s0": 2.0,        # minimum gap (m)
                "alpha": 1.0,     # sensitivity coefficient
                "beta": 0.5       # slope parameter
            }
        else:
            raise ValueError(f"Unknown HDV model: {self.model}")

        for k, v in defaults.items():
            self.params.setdefault(k, v)

    def desired_gap_IDM(self, v, v_front):
        s0 = self.params["s0"]
        T = self.params["T"]
        a_max = self.params["a_max"]
        b = self.params["b"]
        gap = s0 + v * T + v * (v - v_front) / (2 * np.sqrt(a_max * b))
        return gap

    def acceleration(self, s_front, v_front):
        if self.model == "IDM":
            s_star = self.desired_gap_IDM(self.v, v_front)
            a_max = self.params["a_max"]
            v0 = self.params["v0"]
            delta = self.params["delta"]
            a = a_max * (1 - (self.v / v0)**delta - (s_star / s_front)**2)
            self.a = float(a)
        elif self.model == "OVM":
            s0 = self.params["s0"]
            v_max = self.params["v_max"]
            alpha = self.params["alpha"]
            beta = self.params["beta"]
            v_des = v_max * (np.tanh(beta * (s_front - s0)) + np.tanh(beta * s0))
            self.a = float(alpha * (v_des - self.v))
        else:
            self.a = 0.0
        return self.a

    def update(self, dt):
        self.v += self.a * dt
        self.v = max(self.v, 0)  # speed cannot be negative
        self.x += self.v * dt + 0.5 * self.a * dt**2
