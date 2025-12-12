"""
Cooperative Adaptive Cruise Control (CACC) implementation.
Extends ACC by incorporating acceleration feedback from leading vehicle.
"""
import logging
from typing import Optional
from config import CACCConfig

logger = logging.getLogger(__name__)


class CACC:
    """
    Cooperative Adaptive Cruise Control controller.
    
    Extends ACC by adding acceleration feedforward term from the leading vehicle,
    enabling smoother and more responsive platooning behavior.
    """
    
    def __init__(self, k_p=0.2, k_v=0.4, k_a=0.2, s0=2.0, T=0.1,
                 max_accel=3.0, max_decel=-5.0, config: Optional[CACCConfig] = None):
        """
        Initialize CACC controller.
        
        :param k_p: Position gain
        :param k_v: Velocity gain
        :param k_a: Acceleration gain
        :param s0: Minimum spacing (m)
        :param T: Time headway (s)
        :param max_accel: Maximum acceleration (m/s^2)
        :param max_decel: Maximum deceleration (m/s^2)
        :param config: Optional CACCConfig object (overrides individual parameters)
        """
        if config is not None:
            config.validate()
            self.k_p = config.k_p
            self.k_v = config.k_v
            self.k_a = config.k_a
            self.s0 = config.s0
            self.T = config.T
            self.max_accel = config.max_accel
            self.max_decel = config.max_decel
        else:
            self.k_p = k_p
            self.k_v = k_v
            self.k_a = k_a
            self.s0 = s0
            self.T = T
            self.max_accel = max_accel
            self.max_decel = max_decel
            
            # Validate parameters
            if self.k_p <= 0 or self.k_v <= 0 or self.k_a <= 0:
                raise ValueError("Gains k_p, k_v, and k_a must be positive")
            if self.s0 < 0:
                raise ValueError("Minimum spacing s0 must be non-negative")
            if self.T <= 0:
                raise ValueError("Time headway T must be positive")
            if self.max_accel <= 0:
                raise ValueError("max_accel must be positive")
            if self.max_decel >= 0:
                raise ValueError("max_decel must be negative")
        
        logger.info(f"CACC initialized: k_p={self.k_p}, k_v={self.k_v}, k_a={self.k_a}, "
                   f"s0={self.s0}, T={self.T}, limits=[{self.max_decel}, {self.max_accel}]")

    def desired_gap(self, v_self: float) -> float:
        """
        Calculate desired gap to leading vehicle.
        
        :param v_self: Current velocity of self vehicle (m/s)
        :return: Desired gap (m)
        """
        if v_self < 0:
            logger.warning(f"Negative velocity detected: {v_self}, clamping to 0")
            v_self = 0.0
        
        gap = self.s0 + self.T * v_self
        return gap

    def compute_accel(self, v_self: float, s_front: float, v_front: float, 
                     a_front: float = 0.0) -> float:
        """
        Compute acceleration command.
        
        :param v_self: Current velocity of self vehicle (m/s)
        :param s_front: Current gap to front vehicle (m)
        :param v_front: Current velocity of front vehicle (m/s)
        :param a_front: Current acceleration of front vehicle (m/s^2)
        :return: Acceleration command (m/s^2), clamped to [max_decel, max_accel]
        """
        try:
            # Input validation
            if v_self < 0:
                logger.warning(f"Negative self velocity: {v_self}, clamping to 0")
                v_self = 0.0
            
            if s_front < 0:
                logger.error(f"Negative gap detected: {s_front}. This indicates collision risk!")
                raise ValueError(f"Gap s_front must be non-negative, got {s_front}")
            
            if v_front < 0:
                logger.warning(f"Negative front velocity: {v_front}, clamping to 0")
                v_front = 0.0
            
            # Validate front acceleration (should be within reasonable bounds)
            if abs(a_front) > 10.0:  # Unrealistic acceleration
                logger.warning(f"Unrealistic front acceleration: {a_front}, clamping")
                a_front = max(-10.0, min(10.0, a_front))
            
            # Check for dangerously small gap
            if s_front < self.s0:
                logger.warning(f"Gap {s_front} is below minimum spacing {self.s0}")
            
            # Compute desired gap and errors
            s_des = self.desired_gap(v_self)
            gap_error = s_front - s_des
            vel_error = v_front - v_self
            
            # Compute acceleration with feedforward term
            accel = (self.k_p * gap_error +
                    self.k_v * vel_error +
                    self.k_a * a_front)
            
            # Apply physical limits
            accel = max(self.max_decel, min(self.max_accel, accel))
            
            # Log if limits were applied
            raw_accel = self.k_p * gap_error + self.k_v * vel_error + self.k_a * a_front
            if raw_accel != accel:
                logger.debug(f"Acceleration clamped from {raw_accel:.3f} to {accel:.3f}")
            
            return float(accel)
            
        except Exception as e:
            logger.error(f"Error in compute_accel: {e}", exc_info=True)
            # Return safe deceleration in case of error
            logger.warning("Returning maximum deceleration due to error")
            return self.max_decel
