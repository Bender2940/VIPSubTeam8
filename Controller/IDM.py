"""
Intelligent Driver Model (IDM) for simulating car-following behavior.
"""
import logging
from typing import Optional
from config import IDMConfig

logger = logging.getLogger(__name__)


class IDMController:
    """
    Intelligent Driver Model (IDM) for simulating car-following behavior.
    
    Uses a physics-based model that combines free-road acceleration with
    braking deceleration based on desired spacing.
    """

    def __init__(self, v0=30.0, T=1.5, a=1.0, b=1.5, s0=2.0, delta=4,
                 max_accel=3.0, max_decel=-5.0, config: Optional[IDMConfig] = None):
        """
        Initialize IDM parameters.
        
        :param v0: Desired velocity (m/s)
        :param T: Safe time headway (s)
        :param a: Maximum acceleration (m/s^2)
        :param b: Comfortable deceleration (m/s^2)
        :param s0: Minimum distance (m)
        :param delta: Acceleration exponent
        :param max_accel: Maximum acceleration limit (m/s^2)
        :param max_decel: Maximum deceleration limit (m/s^2)
        :param config: Optional IDMConfig object (overrides individual parameters)
        """
        if config is not None:
            config.validate()
            self.v0 = config.v0
            self.T = config.T
            self.a = config.a
            self.b = config.b
            self.s0 = config.s0
            self.delta = config.delta
            self.max_accel = config.max_accel
            self.max_decel = config.max_decel
        else:
            self.v0 = v0
            self.T = T
            self.a = a
            self.b = b
            self.s0 = s0
            self.delta = delta
            self.max_accel = max_accel
            self.max_decel = max_decel
            
            # Validate parameters
            if self.v0 <= 0:
                raise ValueError("Desired velocity v0 must be positive")
            if self.T <= 0:
                raise ValueError("Time headway T must be positive")
            if self.a <= 0:
                raise ValueError("Maximum acceleration a must be positive")
            if self.b <= 0:
                raise ValueError("Comfortable deceleration b must be positive")
            if self.s0 < 0:
                raise ValueError("Minimum distance s0 must be non-negative")
            if self.delta <= 0:
                raise ValueError("Acceleration exponent delta must be positive")
            if self.max_accel <= 0:
                raise ValueError("max_accel must be positive")
            if self.max_decel >= 0:
                raise ValueError("max_decel must be negative")
        
        logger.info(f"IDM initialized: v0={self.v0}, T={self.T}, a={self.a}, "
                   f"b={self.b}, s0={self.s0}, delta={self.delta}, "
                   f"limits=[{self.max_decel}, {self.max_accel}]")

    def calculate_acceleration(self, v: float, s: float, delta_v: float) -> float:
        """
        Calculate acceleration using IDM formula.
        
        :param v: Current velocity of the vehicle (m/s)
        :param s: Gap to the leading vehicle (m)
        :param delta_v: Velocity difference to the leading vehicle (v - v_lead) (m/s)
        :return: Acceleration (m/s^2), clamped to [max_decel, max_accel]
        """
        try:
            # Input validation
            if v < 0:
                logger.warning(f"Negative velocity detected: {v}, clamping to 0")
                v = 0.0
            
            if s < 0:
                logger.error(f"Negative gap detected: {s}. This indicates collision risk!")
                raise ValueError(f"Gap s must be non-negative, got {s}")
            
            # Check for dangerously small gap
            if s < self.s0:
                logger.warning(f"Gap {s} is below minimum distance {self.s0}")
            
            # Prevent division by zero and handle very small gaps
            min_gap = max(s, 0.01)
            
            # Calculate desired spacing
            # s_star = s0 + v*T + (v*delta_v) / (2*sqrt(a*b))
            sqrt_ab = (self.a * self.b) ** 0.5
            if sqrt_ab <= 0:
                logger.error("Invalid sqrt(a*b) calculation")
                return self.max_decel
            
            interaction_term = (v * delta_v) / (2 * sqrt_ab)
            s_star = self.s0 + max(0, v * self.T + interaction_term)
            
            # Calculate acceleration using IDM formula
            # acc = a * [1 - (v/v0)^delta - (s_star/s)^2]
            if self.v0 <= 0:
                logger.error("Invalid desired velocity v0")
                return self.max_decel
            
            free_road_term = 1 - (v / self.v0) ** self.delta
            interaction_braking = (s_star / min_gap) ** 2
            
            acc = self.a * (free_road_term - interaction_braking)
            
            # Apply physical limits
            acc = max(self.max_decel, min(self.max_accel, acc))
            
            # Log if limits were applied
            raw_acc = self.a * (free_road_term - interaction_braking)
            if raw_acc != acc:
                logger.debug(f"Acceleration clamped from {raw_acc:.3f} to {acc:.3f}")
            
            return float(acc)
            
        except Exception as e:
            logger.error(f"Error in calculate_acceleration: {e}", exc_info=True)
            # Return safe deceleration in case of error
            logger.warning("Returning maximum deceleration due to error")
            return self.max_decel