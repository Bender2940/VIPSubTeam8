class IDMController:
    """
    Intelligent Driver Model (IDM) for simulating car-following behavior.
    """

    def __init__(self, v0=30.0, T=1.5, a=1.0, b=1.5, s0=2.0, delta=4):
        """
        Initialize IDM parameters.
        :param v0: Desired velocity (m/s)
        :param T: Safe time headway (s)
        :param a: Maximum acceleration (m/s^2)
        :param b: Comfortable deceleration (m/s^2)
        :param s0: Minimum distance (m)
        :param delta: Acceleration exponent
        """
        self.v0 = v0
        self.T = T
        self.a = a
        self.b = b
        self.s0 = s0
        self.delta = delta

    def calculate_acceleration(self, v, s, delta_v):
        """
        Calculate acceleration using IDM formula.
        :param v: Current velocity of the vehicle (m/s)
        :param s: Gap to the leading vehicle (m)
        :param delta_v: Velocity difference to the leading vehicle (v - v_lead) (m/s)
        :return: Acceleration (m/s^2)
        """
        s_star = self.s0 + max(0, v * self.T + (v * delta_v) / (2 * (self.a * self.b) ** 0.5))
        acc = self.a * (1 - (v / self.v0) ** self.delta - (s_star / max(s, 0.01)) ** 2)
        return acc