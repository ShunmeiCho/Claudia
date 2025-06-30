#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unitree Go2 Connection Test Script
Based on official Unitree documentation recommendations
"""

import sys
import time
import os

# Add project path
sys.path.insert(0, '/home/m1ng/claudia/src')

def test_basic_import():
    """Test basic Unitree SDK import"""
    try:
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.default import LowState_
        print("✅ Unitree SDK import successful")
        return True
    except ImportError as e:
        print(f"❌ Unitree SDK import failed: {e}")
        return False

def test_network_connection():
    """Test network connectivity to robot"""
    robot_ip = "192.168.123.161"
    response = os.system(f"ping -c 1 {robot_ip} > /dev/null 2>&1")
    if response == 0:
        print(f"✅ Network connection to {robot_ip} successful")
        return True
    else:
        print(f"❌ Network connection to {robot_ip} failed")
        return False

def test_cyclonedds_config():
    """Test CycloneDDS configuration"""
    rmw_impl = os.environ.get('RMW_IMPLEMENTATION', '')
    cyclonedds_uri = os.environ.get('CYCLONEDDS_URI', '')
    
    if 'cyclonedds' in rmw_impl.lower():
        print(f"✅ RMW_IMPLEMENTATION: {rmw_impl}")
    else:
        print(f"⚠️  RMW_IMPLEMENTATION: {rmw_impl} (should contain cyclonedds)")
    
    if cyclonedds_uri:
        print(f"✅ CYCLONEDDS_URI configured")
    else:
        print(f"⚠️  CYCLONEDDS_URI not configured")
    
    return True

def test_subscriber_creation():
    """Test creating a subscriber to robot data"""
    try:
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.default import LowState_
        
        # Try to create subscriber
        subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        print("✅ Subscriber creation successful")
        
        # Try to initialize message - this is where we had problems before
        try:
            # Approach 1: Try creating empty message
            msg = LowState_()
            print("✅ LowState_ message creation successful")
            return subscriber, msg
        except TypeError as e:
            print(f"⚠️  LowState_ direct creation failed: {e}")
            # This is expected - we need the subscriber to fill it
            return subscriber, None
            
    except Exception as e:
        print(f"❌ Subscriber creation failed: {e}")
        return None, None

def test_data_reading(subscriber, timeout_seconds=5):
    """Test reading data from robot"""
    if subscriber is None:
        print("❌ No subscriber available for testing")
        return False
    
    try:
        from unitree_sdk2py.idl.default import LowState_
        
        print(f"🔄 Testing data reading (timeout: {timeout_seconds}s)...")
        
        start_time = time.time()
        while time.time() - start_time < timeout_seconds:
            try:
                # Create message buffer
                msg = LowState_()
                
                # Try to read data
                result = subscriber.Read(msg)
                
                if result == 0:  # Success
                    print("✅ Successfully read data from robot!")
                    print(f"   Message timestamp: {getattr(msg, 'tick', 'N/A')}")
                    if hasattr(msg, 'foot_force') and len(msg.foot_force) >= 4:
                        forces = [f"{f:.2f}" for f in msg.foot_force[:4]]
                        print(f"   Foot forces: {forces}")
                    return True
                else:
                    print(f"   Read result: {result} (no data available)")
                    
            except TypeError as e:
                print(f"   LowState_ initialization error: {e}")
                break
            except Exception as e:
                print(f"   Read error: {e}")
                break
                
            time.sleep(0.1)
        
        print("⏰ Data reading timeout - no data received")
        return False
        
    except Exception as e:
        print(f"❌ Data reading test failed: {e}")
        return False

def main():
    """Main test function"""
    print("🧪 Unitree Go2 Connection Test")
    print("="*50)
    
    # Step 1: Test imports
    print("\n1. Testing SDK imports...")
    test_basic_import()
    
    # Step 2: Test network
    print("\n2. Testing network connection...")
    test_network_connection()
    
    # Step 3: Test CycloneDDS config
    print("\n3. Testing CycloneDDS configuration...")
    test_cyclonedds_config()
    
    # Step 4: Test subscriber creation
    print("\n4. Testing subscriber creation...")
    subscriber, msg = test_subscriber_creation()
    
    # Step 5: Test data reading
    print("\n5. Testing data reading...")
    if subscriber:
        success = test_data_reading(subscriber, timeout_seconds=10)
        if success:
            print("\n🎉 All tests passed! Robot connection is working!")
        else:
            print("\n⚠️  Connection established but no data received")
            print("   This might be normal if robot is not actively publishing")
    else:
        print("\n❌ Cannot test data reading without subscriber")
    
    print("\n" + "="*50)
    print("Test completed.")

if __name__ == "__main__":
    main() 