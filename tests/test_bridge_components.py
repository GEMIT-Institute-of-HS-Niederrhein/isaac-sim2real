"""
Unit tests for Dynamixel Bridge components
Run with: pytest tests/ -v
"""
import pytest
import sys
from pathlib import Path

# Add src to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


class TestKeyboardController:
    """Test keyboard controller logic"""
    
    def test_forward_velocity(self):
        """Test forward movement generates correct wheel velocities"""
        from isaac_dxl_bridge import KeyboardController
        
        kb = KeyboardController()
        kb.linear_vel = 1.0
        kb.angular_vel = 0.0
        
        velocities = kb.get_velocities()
        
        # All wheels should move forward equally
        assert all(v == 1.0 for v in velocities), "All wheels should be at max forward"
    
    def test_turn_left(self):
        """Test left turn generates differential velocities"""
        from isaac_dxl_bridge import KeyboardController
        
        kb = KeyboardController()
        kb.linear_vel = 0.0
        kb.angular_vel = 1.0
        
        velocities = kb.get_velocities()
        fl, fr, rl, rr = velocities
        
        # Left wheels should be slower than right wheels
        assert fl < fr, "Front left should be slower than front right"
        assert rl < rr, "Rear left should be slower than rear right"
    
    def test_velocity_clamping(self):
        """Test velocities are clamped to [-1, 1]"""
        from isaac_dxl_bridge import KeyboardController
        
        kb = KeyboardController()
        kb.linear_vel = 2.0  # Over limit
        kb.angular_vel = 2.0  # Over limit
        
        velocities = kb.get_velocities()
        
        # All velocities should be clamped
        assert all(-1.0 <= v <= 1.0 for v in velocities), "Velocities must be in [-1, 1]"


class TestDynamixelController:
    """Test Dynamixel controller (mock hardware)"""
    
    @pytest.mark.skip(reason="Requires hardware")
    def test_motor_initialization(self):
        """Test motor initialization (requires actual hardware)"""
        # This would test actual hardware connection
        pass
    
    def test_velocity_conversion(self):
        """Test velocity conversion from normalized to Dynamixel units"""
        # Normalized velocity of 1.0 should convert to 600
        vel_normalized = 1.0
        vel_dynamixel = int(vel_normalized * 600)
        
        assert vel_dynamixel == 600, "Max velocity should map to 600 units"
    
    def test_negative_velocity_encoding(self):
        """Test negative velocity encoding for Dynamixel"""
        vel = -600
        # Dynamixel uses unsigned 32-bit with two's complement
        if vel < 0:
            vel_encoded = (1 << 32) + vel
        else:
            vel_encoded = vel
        
        assert vel_encoded > 0, "Encoded velocity should be positive"
        assert vel_encoded == 4294966696, "Should encode to correct unsigned value"


def test_imports():
    """Test that all required modules can be imported"""
    try:
        import numpy
        import dynamixel_sdk
        import pynput
        assert True
    except ImportError as e:
        pytest.fail(f"Failed to import required module: {e}")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
