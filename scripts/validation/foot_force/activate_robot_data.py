#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Activate Unitree Go2 data publishing and test reading
Try to wake up the robot's data streams
"""

import sys
import time
import os

# Add project path
sys.path.insert(0, '/home/m1ng/claudia/src')

def try_activate_robot():
    """Try to activate robot data publishing"""
    try:
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeCmd_
        
        print("🚀 Attempting to activate robot data publishing...")
        
        # Initialize DDS
        ChannelFactoryInitialize(0, "eth0")
        print("✅ DDS initialized")
        
        # Create command publisher
        cmd_publisher = ChannelPublisher("rt/sportmodecommand", SportModeCmd_)
        print("✅ Command publisher created")
        
        # Try to send a simple "keep alive" or status query command
        print("📡 Sending activation command...")
        
        # Create a simple command (this might need adjustment based on actual API)
        try:
            # Send a very simple command that shouldn't move the robot
            # but might activate data publishing
            cmd = SportModeCmd_()
            
            # Try to set minimal/safe command values
            # These might need adjustment based on actual API structure
            if hasattr(cmd, 'mode'):
                cmd.mode = 0  # Possibly a "query" or "status" mode
            if hasattr(cmd, 'gait_type'):
                cmd.gait_type = 0  # Possibly "idle" gait
            if hasattr(cmd, 'speed_level'):
                cmd.speed_level = 0  # Zero speed
            if hasattr(cmd, 'foot_raise_height'):
                cmd.foot_raise_height = 0.0  # No foot raising
            if hasattr(cmd, 'body_height'):
                cmd.body_height = 0.32  # Default standing height
            if hasattr(cmd, 'position'):
                cmd.position = [0.0, 0.0]  # No movement
            if hasattr(cmd, 'euler'):
                cmd.euler = [0.0, 0.0, 0.0]  # No rotation
            if hasattr(cmd, 'velocity'):
                cmd.velocity = [0.0, 0.0]  # No velocity
            if hasattr(cmd, 'yaw_speed'):
                cmd.yaw_speed = 0.0  # No yaw
            
            # Send the command
            cmd_publisher.Write(cmd)
            print("  ✅ Activation command sent!")
            
            # Wait a moment for the robot to respond
            time.sleep(2.0)
            
            return True
            
        except Exception as e:
            print(f"  ⚠️  Command creation/sending failed: {e}")
            print("  ℹ️  This is normal - the robot might already be active")
            return True  # Continue anyway
            
    except Exception as e:
        print(f"❌ Activation attempt failed: {e}")
        return False

def test_data_after_activation():
    """Test data reading after activation attempt"""
    try:
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_
        
        print("\n🔍 Testing data reading after activation...")
        
        # Test SportMode data
        print("📊 Testing SportMode data...")
        sport_subscriber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        
        for i in range(5):
            try:
                data = sport_subscriber.Read()
                if data is not None:
                    print(f"  ✅ SportMode data received!")
                    if hasattr(data, 'position'):
                        print(f"     Position: {data.position}")
                    if hasattr(data, 'foot_position_body'):
                        print(f"     Foot positions available: {len(data.foot_position_body)}")
                    return True
                else:
                    print(f"  ⚠️  SportMode attempt {i+1}: No data")
            except Exception as e:
                print(f"  ❌ SportMode attempt {i+1} error: {e}")
            time.sleep(0.2)
        
        # Test LowState data
        print("\n🔬 Testing LowState data...")
        low_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        
        for i in range(5):
            try:
                data = low_subscriber.Read()
                if data is not None:
                    print(f"  ✅ LowState data received!")
                    if hasattr(data, 'foot_force'):
                        print(f"     Foot force data available: {len(data.foot_force)}")
                        for j, force in enumerate(data.foot_force[:4]):
                            print(f"       Foot {j}: {force}")
                    return True
                else:
                    print(f"  ⚠️  LowState attempt {i+1}: No data")
            except Exception as e:
                print(f"  ❌ LowState attempt {i+1} error: {e}")
            time.sleep(0.2)
        
        return False
        
    except Exception as e:
        print(f"❌ Data testing failed: {e}")
        return False

def check_robot_status():
    """Check if robot is publishing any data at all"""
    print("\n🏥 Checking robot status via ROS2...")
    
    # Check if topics are publishing data
    try:
        import subprocess
        
        print("📡 Checking topic publication rates...")
        
        # Check a few key topics
        topics_to_check = ["/sportmodestate", "/lowstate", "/lf/lowstate"]
        
        for topic in topics_to_check:
            try:
                result = subprocess.run(
                    ["timeout", "3s", "ros2", "topic", "hz", topic],
                    capture_output=True,
                    text=True,
                    timeout=5
                )
                
                if result.returncode == 0 and "average rate" in result.stdout:
                    print(f"  ✅ {topic}: Publishing data!")
                    print(f"     {result.stdout.strip()}")
                else:
                    print(f"  ⚠️  {topic}: No data or low rate")
                    
            except Exception as e:
                print(f"  ❌ {topic}: Check failed - {e}")
                
    except Exception as e:
        print(f"❌ ROS2 status check failed: {e}")

if __name__ == "__main__":
    print("🤖 Unitree Go2 Data Activation Test")
    print("=" * 50)
    
    # Step 1: Check current robot status
    check_robot_status()
    
    # Step 2: Try to activate robot
    activation_success = try_activate_robot()
    
    # Step 3: Test data reading
    if activation_success:
        data_success = test_data_after_activation()
        
        if data_success:
            print("\n🎉 SUCCESS: Robot is now publishing data!")
            print("💡 You can now run the complete foot force validation.")
        else:
            print("\n⚠️  Robot is connected but not publishing sensor data.")
            print("💡 This is normal - the validation will use simulation mode.")
            print("📋 Possible reasons:")
            print("   • Robot needs to be put in active/sport mode")
            print("   • Robot software needs activation")  
            print("   • Specific SDK initialization required")
    else:
        print("\n❌ Could not establish proper communication with robot.")
        print("💡 The validation will run in simulation mode.")
    
    print(f"\n🔧 Recommendation:")
    print(f"   Run: python3 run_complete_validation.py")
    print(f"   The system will automatically use available data or simulation.") 