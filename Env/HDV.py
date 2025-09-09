"""
Human Driven Vehicle (HDV) Environment Module

This module contains the environment and behavior models for human-driven vehicles
in the mixed-traffic platoon simulation.
"""


class HDVEnvironment:
    """
    Human Driven Vehicle Environment class.
    
    This class represents the environment and behavior model for human-driven vehicles
    in a mixed-traffic scenario.
    """
    
    def __init__(self):
        """Initialize the HDV environment."""
        self.vehicle_type = "HDV"
        self.is_autonomous = False
        
    def update(self):
        """Update the HDV environment state."""
        pass
        
    def get_state(self):
        """Get current state of the HDV environment."""
        return {
            "vehicle_type": self.vehicle_type,
            "is_autonomous": self.is_autonomous
        }