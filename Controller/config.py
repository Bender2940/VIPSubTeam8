"""
Configuration system for vehicle controllers.
Provides default parameters and validation for ACC, CACC, and IDM controllers.
"""
from dataclasses import dataclass
from typing import Optional


@dataclass
class ACCConfig:
    """Configuration for Adaptive Cruise Control."""
    k_p: float = 0.2  # Position gain
    k_v: float = 0.4  # Velocity gain
    s0: float = 2.0   # Minimum spacing (m)
    T: float = 0.1    # Time headway (s)
    max_accel: float = 3.0   # Maximum acceleration (m/s^2)
    max_decel: float = -5.0  # Maximum deceleration (m/s^2)
    
    def validate(self) -> None:
        """Validate configuration parameters."""
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


@dataclass
class CACCConfig:
    """Configuration for Cooperative Adaptive Cruise Control."""
    k_p: float = 0.2  # Position gain
    k_v: float = 0.4  # Velocity gain
    k_a: float = 0.2  # Acceleration gain
    s0: float = 2.0   # Minimum spacing (m)
    T: float = 0.1    # Time headway (s)
    max_accel: float = 3.0   # Maximum acceleration (m/s^2)
    max_decel: float = -5.0  # Maximum deceleration (m/s^2)
    
    def validate(self) -> None:
        """Validate configuration parameters."""
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


@dataclass
class IDMConfig:
    """Configuration for Intelligent Driver Model."""
    v0: float = 30.0  # Desired velocity (m/s)
    T: float = 1.5    # Safe time headway (s)
    a: float = 1.0    # Maximum acceleration (m/s^2)
    b: float = 1.5    # Comfortable deceleration (m/s^2)
    s0: float = 2.0   # Minimum distance (m)
    delta: float = 4  # Acceleration exponent
    max_accel: float = 3.0   # Maximum acceleration limit (m/s^2)
    max_decel: float = -5.0  # Maximum deceleration limit (m/s^2)
    
    def validate(self) -> None:
        """Validate configuration parameters."""
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


def load_config_from_dict(config_type: str, config_dict: dict):
    """
    Load configuration from dictionary.
    
    :param config_type: Type of config ('ACC', 'CACC', or 'IDM')
    :param config_dict: Dictionary of configuration parameters
    :return: Configuration object
    """
    if config_type.upper() == 'ACC':
        config = ACCConfig(**config_dict)
    elif config_type.upper() == 'CACC':
        config = CACCConfig(**config_dict)
    elif config_type.upper() == 'IDM':
        config = IDMConfig(**config_dict)
    else:
        raise ValueError(f"Unknown config type: {config_type}")
    
    config.validate()
    return config

