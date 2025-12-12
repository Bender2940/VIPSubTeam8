"""
Integration and comparison utilities for vehicle controllers.
Provides tools to compare and analyze different controller behaviors.
"""
import logging
import math
from typing import List, Dict, Tuple, Optional
try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False
    # Fallback functions for when numpy is not available
    def np_mean(arr):
        return sum(arr) / len(arr) if arr else 0.0
    def np_std(arr):
        if not arr:
            return 0.0
        mean = np_mean(arr)
        variance = sum((x - mean) ** 2 for x in arr) / len(arr)
        return math.sqrt(variance)
    def np_min(arr):
        return min(arr) if arr else 0.0
    def np_max(arr):
        return max(arr) if arr else 0.0
    def np_sqrt(x):
        return math.sqrt(x)

from ACC import ACC
from CACC import CACC
from IDM import IDMController
from config import ACCConfig, CACCConfig, IDMConfig

logger = logging.getLogger(__name__)


class ControllerComparison:
    """
    Utility class for comparing multiple controllers under the same conditions.
    """
    
    def __init__(self):
        """Initialize the comparison utility."""
        self.results: Dict[str, List[float]] = {}
        self.controllers: Dict[str, object] = {}
    
    def add_controller(self, name: str, controller):
        """
        Add a controller to the comparison.
        
        :param name: Name identifier for the controller
        :param controller: Controller instance (ACC, CACC, or IDMController)
        """
        if not isinstance(controller, (ACC, CACC, IDMController)):
            raise ValueError("Controller must be ACC, CACC, or IDMController instance")
        
        self.controllers[name] = controller
        self.results[name] = []
        logger.info(f"Added controller: {name}")
    
    def simulate_scenario(self, scenario: Dict, dt: float = 0.1, duration: float = 10.0):
        """
        Simulate a scenario with all added controllers.
        
        :param scenario: Dictionary with keys:
            - 'v_self': List of self vehicle velocities over time
            - 's_front': List of gaps to front vehicle over time
            - 'v_front': List of front vehicle velocities over time
            - 'a_front': (Optional) List of front vehicle accelerations over time
        :param dt: Time step (s)
        :param duration: Simulation duration (s)
        """
        n_steps = int(duration / dt)
        
        # Validate scenario data
        required_keys = ['v_self', 's_front', 'v_front']
        for key in required_keys:
            if key not in scenario:
                raise ValueError(f"Missing required scenario key: {key}")
            if len(scenario[key]) < n_steps:
                raise ValueError(f"Scenario {key} has insufficient data points")
        
        has_a_front = 'a_front' in scenario and len(scenario['a_front']) >= n_steps
        
        # Reset results
        for name in self.controllers:
            self.results[name] = []
        
        logger.info(f"Simulating scenario for {duration}s with dt={dt}s")
        
        # Simulate each time step
        for i in range(n_steps):
            v_self = scenario['v_self'][i]
            s_front = scenario['s_front'][i]
            v_front = scenario['v_front'][i]
            a_front = scenario['a_front'][i] if has_a_front else 0.0
            
            # Compute acceleration for each controller
            for name, controller in self.controllers.items():
                try:
                    if isinstance(controller, ACC):
                        accel = controller.compute_accel(v_self, s_front, v_front)
                    elif isinstance(controller, CACC):
                        accel = controller.compute_accel(v_self, s_front, v_front, a_front)
                    elif isinstance(controller, IDMController):
                        delta_v = v_self - v_front
                        accel = controller.calculate_acceleration(v_self, s_front, delta_v)
                    else:
                        raise ValueError(f"Unknown controller type: {type(controller)}")
                    
                    self.results[name].append(accel)
                    
                except Exception as e:
                    logger.error(f"Error computing acceleration for {name} at step {i}: {e}")
                    self.results[name].append(0.0)  # Default to zero acceleration on error
    
    def get_statistics(self) -> Dict[str, Dict[str, float]]:
        """
        Compute statistics for each controller's acceleration outputs.
        
        :return: Dictionary mapping controller names to their statistics
        """
        stats = {}
        for name, accelerations in self.results.items():
            if not accelerations:
                stats[name] = {}
                continue
            
            if HAS_NUMPY:
                acc_array = np.array(accelerations)
                mean_val = float(np.mean(acc_array))
                std_val = float(np.std(acc_array))
                min_val = float(np.min(acc_array))
                max_val = float(np.max(acc_array))
                rms_val = float(np.sqrt(np.mean(acc_array ** 2)))
            else:
                mean_val = np_mean(accelerations)
                std_val = np_std(accelerations)
                min_val = np_min(accelerations)
                max_val = np_max(accelerations)
                rms_val = math.sqrt(sum(x ** 2 for x in accelerations) / len(accelerations))
            
            stats[name] = {
                'mean': mean_val,
                'std': std_val,
                'min': min_val,
                'max': max_val,
                'rms': rms_val,
            }
        
        return stats
    
    def compare_controllers(self, scenario: Dict, dt: float = 0.1, 
                           duration: float = 10.0) -> Dict:
        """
        Run simulation and return comprehensive comparison results.
        
        :param scenario: Scenario dictionary (see simulate_scenario)
        :param dt: Time step (s)
        :param duration: Simulation duration (s)
        :return: Dictionary with comparison results
        """
        self.simulate_scenario(scenario, dt, duration)
        stats = self.get_statistics()
        
        return {
            'statistics': stats,
            'time_series': self.results.copy(),
            'controllers': list(self.controllers.keys())
        }


def create_standard_scenarios() -> Dict[str, Dict]:
    """
    Create standard test scenarios for controller evaluation.
    
    :return: Dictionary of scenario names to scenario data
    """
    dt = 0.1
    duration = 20.0
    n_steps = int(duration / dt)
    
    # Create time array
    if HAS_NUMPY:
        time = np.linspace(0, duration, n_steps).tolist()
    else:
        time = [i * dt for i in range(n_steps)]
    
    scenarios = {}
    
    # Scenario 1: Constant speed following
    scenarios['constant_speed'] = {
        'v_self': [20.0] * n_steps,
        's_front': [30.0] * n_steps,
        'v_front': [20.0] * n_steps,
        'a_front': [0.0] * n_steps,
    }
    
    # Scenario 2: Leading vehicle deceleration
    scenarios['deceleration'] = {
        'v_self': [25.0] * n_steps,
        's_front': [40.0 - 0.5 * t for t in time],
        'v_front': [max(0, 25.0 - 2.0 * t) for t in time],
        'a_front': [-2.0] * n_steps,
    }
    
    # Scenario 3: Leading vehicle acceleration
    scenarios['acceleration'] = {
        'v_self': [15.0] * n_steps,
        's_front': [30.0 + 0.3 * t for t in time],
        'v_front': [min(30.0, 15.0 + 1.5 * t) for t in time],
        'a_front': [1.5] * n_steps,
    }
    
    # Scenario 4: Cut-in scenario (gap suddenly decreases)
    scenarios['cut_in'] = {
        'v_self': [20.0] * n_steps,
        's_front': [max(5.0, 50.0 - 2.0 * t) if t < 5.0 else 5.0 + 0.1 * (t - 5.0) for t in time],
        'v_front': [20.0] * n_steps,
        'a_front': [0.0] * n_steps,
    }
    
    # Scenario 5: Oscillating lead vehicle
    scenarios['oscillation'] = {
        'v_self': [20.0] * n_steps,
        's_front': [30.0 + 5.0 * math.sin(0.5 * t) for t in time],
        'v_front': [20.0 + 2.0 * math.cos(0.5 * t) for t in time],
        'a_front': [-1.0 * math.sin(0.5 * t) for t in time],
    }
    
    return scenarios


def compare_all_controllers(scenario_name: str = 'constant_speed', 
                           custom_scenario: Optional[Dict] = None) -> Dict:
    """
    Compare all three controllers (ACC, CACC, IDM) on a standard scenario.
    
    :param scenario_name: Name of standard scenario to use
    :param custom_scenario: Optional custom scenario dictionary
    :return: Comparison results dictionary
    """
    # Create controllers with default parameters
    acc = ACC()
    cacc = CACC()
    idm = IDMController()
    
    # Create comparison utility
    comparison = ControllerComparison()
    comparison.add_controller('ACC', acc)
    comparison.add_controller('CACC', cacc)
    comparison.add_controller('IDM', idm)
    
    # Get scenario
    if custom_scenario is not None:
        scenario = custom_scenario
    else:
        scenarios = create_standard_scenarios()
        if scenario_name not in scenarios:
            raise ValueError(f"Unknown scenario: {scenario_name}. "
                           f"Available: {list(scenarios.keys())}")
        scenario = scenarios[scenario_name]
    
    # Run comparison
    results = comparison.compare_controllers(scenario, dt=0.1, duration=20.0)
    
    return results


def print_comparison_summary(results: Dict):
    """
    Print a formatted summary of comparison results.
    
    :param results: Results dictionary from compare_controllers
    """
    print("\n" + "="*60)
    print("CONTROLLER COMPARISON SUMMARY")
    print("="*60)
    
    stats = results['statistics']
    
    for controller_name in results['controllers']:
        if controller_name not in stats:
            continue
        
        print(f"\n{controller_name}:")
        print(f"  Mean acceleration:    {stats[controller_name]['mean']:8.3f} m/s²")
        print(f"  Std deviation:       {stats[controller_name]['std']:8.3f} m/s²")
        print(f"  Min acceleration:    {stats[controller_name]['min']:8.3f} m/s²")
        print(f"  Max acceleration:    {stats[controller_name]['max']:8.3f} m/s²")
        print(f"  RMS acceleration:    {stats[controller_name]['rms']:8.3f} m/s²")
    
    print("\n" + "="*60)

