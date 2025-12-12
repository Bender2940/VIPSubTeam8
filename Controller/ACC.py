"""
Adaptive Cruise Control (ACC) implementation.
Maintains desired spacing using position and velocity feedback.
"""
import logging
from typing import Optional
from config import ACCConfig

logger = logging.getLogger(__name__)


class ACC:
    """
    Adaptive Cruise Control controller.
    
    Uses proportional control on gap error and velocity error to maintain
    safe following distance behind a leading vehicle.
    """
    
    def __init__(self, k_p=0.2, k_v=0.4, s0=2.0, T=0.1, 
                 max_accel=3.0, max_decel=-5.0, config: Optional[ACCConfig] = None):
        """
        Initialize ACC controller.
        
        :param k_p: Position gain
        :param k_v: Velocity gain
        :param s0: Minimum spacing (m)
        :param T: Time headway (s)
        :param max_accel: Maximum acceleration (m/s^2)
        :param max_decel: Maximum deceleration (m/s^2)
        :param config: Optional ACCConfig object (overrides individual parameters)
        """
        if config is not None:
            config.validate()
            self.k_p = config.k_p
            self.k_v = config.k_v
            self.s0 = config.s0
            self.T = config.T
            self.max_accel = config.max_accel
            self.max_decel = config.max_decel
        else:
            self.k_p = k_p
            self.k_v = k_v
            self.s0 = s0
            self.T = T
            self.max_accel = max_accel
            self.max_decel = max_decel
            
            # Validate parameters
            if self.k_p <= 0 or self.k_v <= 0:
                raise ValueError("Gains k_p and k_v must be positive")
            if self.s0 < 0:
                raise ValueError("Minimum spacing s0 must be non-negative")
            if self.T <= 0:
                raise ValueError("Time headway T must be positive")
            if self.max_accel <= 0:
                raise ValueError("max_accel must be positive")
            if self.max_decel >= 0:
                raise ValueError("max_decel must be negative")
        
        logger.info(f"ACC initialized: k_p={self.k_p}, k_v={self.k_v}, "
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

    def compute_accel(self, v_self: float, s_front: float, v_front: float) -> float:
        """
        Compute acceleration command.
        
        :param v_self: Current velocity of self vehicle (m/s)
        :param s_front: Current gap to front vehicle (m)
        :param v_front: Current velocity of front vehicle (m/s)
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
            
            # Check for dangerously small gap
            if s_front < self.s0:
                logger.warning(f"Gap {s_front} is below minimum spacing {self.s0}")
            
            # Compute desired gap and errors
            s_des = self.desired_gap(v_self)
            gap_error = s_front - s_des
            vel_error = v_front - v_self
            
            # Compute acceleration
            accel = self.k_p * gap_error + self.k_v * vel_error
            
            # Apply physical limits
            accel = max(self.max_decel, min(self.max_accel, accel))
            
            # Log if limits were applied
            raw_accel = self.k_p * gap_error + self.k_v * vel_error
            if raw_accel != accel:
                logger.debug(f"Acceleration clamped from {raw_accel:.3f} to {accel:.3f}")
            
            return float(accel)
            
        except Exception as e:
            logger.error(f"Error in compute_accel: {e}", exc_info=True)
            # Return safe deceleration in case of error
            logger.warning("Returning maximum deceleration due to error")
            return self.max_decel
