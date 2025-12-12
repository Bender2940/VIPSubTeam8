"""
Demo script showing how to use the controllers with logging and comparison utilities.
"""
import logging
from ACC import ACC
from CACC import CACC
from IDM import IDMController
from config import ACCConfig, CACCConfig, IDMConfig
from comparison_utils import compare_all_controllers, print_comparison_summary, create_standard_scenarios

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

logger = logging.getLogger(__name__)


def demo_basic_usage():
    """Demonstrate basic usage of individual controllers."""
    print("\n" + "="*60)
    print("BASIC CONTROLLER USAGE DEMO")
    print("="*60)
    
    # Create controllers
    acc = ACC()
    cacc = CACC()
    idm = IDMController()
    
    # Test scenario: following at 20 m/s with 30m gap
    v_self = 20.0
    s_front = 30.0
    v_front = 20.0
    a_front = 0.0
    
    print(f"\nScenario: v_self={v_self} m/s, s_front={s_front} m, "
          f"v_front={v_front} m/s, a_front={a_front} m/s²")
    
    accel_acc = acc.compute_accel(v_self, s_front, v_front)
    accel_cacc = cacc.compute_accel(v_self, s_front, v_front, a_front)
    delta_v = v_self - v_front
    accel_idm = idm.calculate_acceleration(v_self, s_front, delta_v)
    
    print(f"\nACC acceleration:  {accel_acc:6.3f} m/s²")
    print(f"CACC acceleration: {accel_cacc:6.3f} m/s²")
    print(f"IDM acceleration:  {accel_idm:6.3f} m/s²")


def demo_with_config():
    """Demonstrate usage with configuration objects."""
    print("\n" + "="*60)
    print("USING CONFIGURATION OBJECTS")
    print("="*60)
    
    # Create custom configuration
    acc_config = ACCConfig(
        k_p=0.3,
        k_v=0.5,
        s0=3.0,
        T=0.15,
        max_accel=4.0,
        max_decel=-6.0
    )
    
    acc = ACC(config=acc_config)
    print(f"\nCreated ACC with custom config:")
    print(f"  k_p={acc.k_p}, k_v={acc.k_v}, s0={acc.s0}, T={acc.T}")
    print(f"  Limits: [{acc.max_decel}, {acc.max_accel}] m/s²")


def demo_comparison():
    """Demonstrate controller comparison utilities."""
    print("\n" + "="*60)
    print("CONTROLLER COMPARISON DEMO")
    print("="*60)
    
    # Compare on constant speed scenario
    print("\nComparing controllers on 'constant_speed' scenario...")
    results = compare_all_controllers('constant_speed')
    print_comparison_summary(results)
    
    # Compare on deceleration scenario
    print("\n\nComparing controllers on 'deceleration' scenario...")
    results = compare_all_controllers('deceleration')
    print_comparison_summary(results)


def demo_error_handling():
    """Demonstrate error handling and validation."""
    print("\n" + "="*60)
    print("ERROR HANDLING DEMO")
    print("="*60)
    
    acc = ACC()
    
    # Test with invalid inputs
    print("\nTesting with edge cases:")
    
    # Negative gap (should raise error or handle gracefully)
    try:
        accel = acc.compute_accel(20.0, -5.0, 20.0)
        print(f"Negative gap handled: acceleration = {accel:.3f} m/s²")
    except Exception as e:
        print(f"Negative gap error: {e}")
    
    # Very small gap
    accel = acc.compute_accel(20.0, 1.0, 20.0)
    print(f"Small gap (1m): acceleration = {accel:.3f} m/s²")
    
    # Negative velocity (should be clamped)
    accel = acc.compute_accel(-5.0, 30.0, 20.0)
    print(f"Negative velocity: acceleration = {accel:.3f} m/s²")


if __name__ == "__main__":
    print("\n" + "="*60)
    print("VEHICLE CONTROLLER DEMONSTRATION")
    print("="*60)
    
    try:
        demo_basic_usage()
        demo_with_config()
        demo_comparison()
        demo_error_handling()
        
        print("\n" + "="*60)
        print("DEMO COMPLETE")
        print("="*60 + "\n")
        
    except Exception as e:
        logger.error(f"Demo failed: {e}", exc_info=True)
        raise

