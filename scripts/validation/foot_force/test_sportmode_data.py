#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test SportMode data reading from Unitree Go2
This data stream is usually more reliable
"""

import sys
import time
import os

# Add project path
_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'src'))

def test_sportmode_data():
    """Test reading SportMode data"""
    try:
        from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
        
        print("🏃 Testing SportMode data reading...")
        
        # Initialize DDS
        ChannelFactoryInitialize(0, "eth0")
        print("✅ DDS initialized")
        
        # Create subscriber for SportMode
        subscriber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        print("✅ SportMode subscriber created")
        
        # Try to read SportMode data
        print("\n📝 Reading SportMode data...")
        success_count = 0
        
        for i in range(10):
            try:
                data = subscriber.Read()
                if data is not None:
                    print(f"  ✅ Read attempt {i+1}: Got SportMode data!")
                    success_count += 1
                    
                    # Print some basic info
                    if hasattr(data, 'position'):
                        print(f"     Position: x={data.position[0]:.3f}, y={data.position[1]:.3f}, z={data.position[2]:.3f}")
                    if hasattr(data, 'velocity'):
                        print(f"     Velocity: vx={data.velocity[0]:.3f}, vy={data.velocity[1]:.3f}, vz={data.velocity[2]:.3f}")
                    if hasattr(data, 'foot_position_body'):
                        print(f"     Foot count: {len(data.foot_position_body)}")
                        for j, foot_pos in enumerate(data.foot_position_body):
                            print(f"       Foot {j}: x={foot_pos[0]:.3f}, y={foot_pos[1]:.3f}, z={foot_pos[2]:.3f}")
                    
                    break
                else:
                    print(f"  ⚠️  Read attempt {i+1}: No data")
                
                time.sleep(0.1)
                
            except Exception as e:
                print(f"  ❌ Read attempt {i+1} failed: {e}")
                time.sleep(0.1)
        
        if success_count > 0:
            print(f"\n🎉 Successfully read SportMode data! ({success_count}/10 attempts)")
            return True
        else:
            print(f"\n❌ Failed to read SportMode data (0/10 attempts)")
            return False
        
    except ImportError as e:
        print(f"❌ Import failed: {e}")
        return False
    except Exception as e:
        print(f"❌ Test failed: {e}")
        return False

def test_lowstate_alternative():
    """Test alternative approaches for LowState data"""
    try:
        from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
        
        print("\n🔧 Testing LowState alternative approaches...")
        
        # Try different channel names
        channel_names = ["rt/lowstate", "lowstate", "lf/lowstate"]
        
        for channel_name in channel_names:
            print(f"\n📡 Testing channel: {channel_name}")
            try:
                subscriber = ChannelSubscriber(channel_name, LowState_)
                print(f"  ✅ Subscriber created for {channel_name}")
                
                # Try to read data
                for i in range(3):
                    try:
                        data = subscriber.Read()
                        if data is not None:
                            print(f"    ✅ Successfully read data from {channel_name}!")
                            if hasattr(data, 'foot_force'):
                                print(f"       Foot force data available: {len(data.foot_force)} elements")
                            return True
                        else:
                            print(f"    ⚠️  No data from {channel_name} (attempt {i+1})")
                    except Exception as e:
                        print(f"    ❌ Read error from {channel_name}: {e}")
                    time.sleep(0.1)
                    
            except Exception as e:
                print(f"  ❌ Failed to create subscriber for {channel_name}: {e}")
        
        return False
        
    except Exception as e:
        print(f"❌ LowState alternative test failed: {e}")
        return False

if __name__ == "__main__":
    # Test SportMode first (usually more reliable)
    sportmode_success = test_sportmode_data()
    
    # Test LowState alternatives
    lowstate_success = test_lowstate_alternative()
    
    print(f"\n📊 Test Results:")
    print(f"   SportMode data: {'✅ Success' if sportmode_success else '❌ Failed'}")
    print(f"   LowState data: {'✅ Success' if lowstate_success else '❌ Failed'}")
    
    if sportmode_success:
        print("\n💡 Recommendation: Robot is connected and publishing data!")
        print("   The issue may be with LowState channel or foot force data format.")
    elif not sportmode_success and not lowstate_success:
        print("\n⚠️  Robot may not be actively publishing data or requires activation.")
        print("   Check if robot is in the correct mode and actively running.") 