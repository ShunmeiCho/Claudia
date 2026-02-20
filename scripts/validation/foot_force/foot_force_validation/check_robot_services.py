#!/usr/bin/env python3
# Check Unitree robot services and topics
import sys
import subprocess
import time
import os
_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'src'))

def check_robot_services():
    """Check if robot services are running"""
    print("🔍 Checking Unitree robot services...")
    
    # Check if robot is accessible via SSH (common way to check robot status)
    print("1. Testing robot accessibility...")
    try:
        result = subprocess.run(["ping", "-c", "1", "192.168.123.161"], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print("✅ Robot network accessible")
        else:
            print("❌ Robot network not accessible")
            return False
    except Exception as e:
        print(f"❌ Network test failed: {e}")
        return False
    
    # Check available DDS topics using Unitree SDK
    print("2. Checking available DDS topics...")
    try:
        from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
        
        # Initialize DDS
        ChannelFactoryInitialize(0, "eth0")
        
        # Try to check what topics are available
        # This is tricky as there's no direct "list topics" in unitree_sdk2py
        # But we can try different known topic names
        
        known_topics = [
            "rt/lowstate",
            "rt/highstate", 
            "rt/sportmodestate",
            "rt/utlidar",
            "rt/img",
            "rt/foot_force"
        ]
        
        for topic in known_topics:
            try:
                # Try to create subscriber for each topic
                from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
                subscriber = ChannelSubscriber(topic, LowState_)
                print(f"   ✅ Topic '{topic}' - subscriber created")
                
                # Try reading once with very short timeout
                result = subscriber.Read(timeout=10)  # 10ms timeout
                if result is not None:
                    print(f"      🎉 Data available on '{topic}'!")
                else:
                    print(f"      ℹ️ No data on '{topic}' (may be normal)")
                    
            except Exception as e:
                print(f"   ❌ Topic '{topic}' failed: {e}")
        
        return True
        
    except Exception as e:
        print(f"❌ DDS topic check failed: {e}")
        return False

def check_robot_manual_start():
    """Provide manual check instructions"""
    print("\n🤖 Manual Robot Status Check Instructions:")
    print("="*50)
    print("Please manually verify on the robot:")
    print("1. Is the robot powered on and fully booted?")
    print("2. Are the robot's status LEDs showing normal operation?")
    print("3. Is the robot in a stable standing position?")
    print("4. Try running this command on the robot (if you have SSH access):")
    print("   systemctl status unitree-robot")
    print("5. Check if robot services are running:")
    print("   ps aux | grep unitree")

if __name__ == "__main__":
    print("🧪 Unitree Robot Service Check")
    print("="*40)
    
    success = check_robot_services()
    
    if not success:
        check_robot_manual_start()
    
    print("\n" + "="*40)
    print("Check completed!")
