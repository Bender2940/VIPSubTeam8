import math

# Optimal Velocity Model (OVM) implementation for vehicle dynamics

class OVMController:
    """
    Implements the Optimal Velocity Model (OVM) for vehicle following behavior.
    """

    def __init__(self, v_max, h_st, h_go, alpha):
        """
        Initialize the OVM parameters.
        :param v_max: Maximum velocity (float)
        :param h_st: Standstill headway (float)
        :param h_go: Go headway (float)
        :param alpha: Sensitivity parameter (float)
        """
        self.v_max = v_max
        self.h_st = h_st
        self.h_go = h_go
        self.alpha = alpha

    def optimal_velocity(self, headway):
        """
        Compute the optimal velocity given the current headway.
        :param headway: Distance to the vehicle in front (float)
        :return: Optimal velocity (float)
        """
        if headway <= self.h_st:
            return 0.0
        elif headway >= self.h_go:
            return self.v_max
        else:
            return self.v_max * 0.5 * (1 - math.cos(math.pi * (headway - self.h_st) / (self.h_go - self.h_st)))

    def calculate_acceleration(self, current_velocity, headway):
        """
        Compute the acceleration based on current velocity and headway.
        :param current_velocity: Current velocity of the vehicle (float)
        :param headway: Distance to the vehicle in front (float)
        :return: Acceleration (float)
        """
        v_opt = self.optimal_velocity(headway)
        return self.alpha * (v_opt - current_velocity)