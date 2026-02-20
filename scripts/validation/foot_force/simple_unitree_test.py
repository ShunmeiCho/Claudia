#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple Unitree SDK Test Script
Tests the correct way to read data from Unitree Go2
"""

import sys
import time
import os

# Add project path
_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'src'))

def test_unitree_sdk_reading():
    """Test different methods to read Unitree data"""
    try:
        from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
        
        print("🔧 Testing Unitree SDK data reading methods...")
        
        # Initialize DDS
        ChannelFactoryInitialize(0, "eth0")
        print("✅ DDS initialized")
        
        # Create subscriber
        subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        print("✅ Subscriber created")
        
        # Method 1: Try Read() without parameters
        print("\n📝 Method 1: subscriber.Read()")
        try:
            for i in range(3):
                data = subscriber.Read()
                if data is not None:
                    print(f"  ✅ Read attempt {i+1}: Got data!")
                    if hasattr(data, 'tick'):
                        print(f"     Tick: {data.tick}")
                    if hasattr(data, 'foot_force'):
                        print(f"     Foot force length: {len(data.foot_force)}")
                    break
                else:
                    print(f"  ⚠️  Read attempt {i+1}: No data")
                time.sleep(0.1)
        except Exception as e:
            print(f"  ❌ Method 1 failed: {e}")
        
        # Method 2: Try Read() with timeout
        print("\n📝 Method 2: subscriber.Read(timeout=1000)")
        try:
            for i in range(3):
                data = subscriber.Read(timeout=1000)
                if data is not None:
                    print(f"  ✅ Read attempt {i+1}: Got data!")
                    break
                else:
                    print(f"  ⚠️  Read attempt {i+1}: No data")
                time.sleep(0.1)
        except Exception as e:
            print(f"  ❌ Method 2 failed: {e}")
        
        # Method 3: Traditional pattern with message parameter
        print("\n📝 Method 3: subscriber.Read(message)")
        try:
            for i in range(3):
                msg = LowState_()
                result = subscriber.Read(msg)
                print(f"  Read attempt {i+1}: Result = {result}")
                if result == 0:  # Success
                    print(f"  ✅ Successfully read data!")
                    if hasattr(msg, 'tick'):
                        print(f"     Tick: {msg.tick}")
                    if hasattr(msg, 'foot_force'):
                        print(f"     Foot force length: {len(msg.foot_force)}")
                        print(f"     First foot force: {msg.foot_force[0] if len(msg.foot_force) > 0 else 'N/A'}")
                    break
                time.sleep(0.1)
        except Exception as e:
            print(f"  ❌ Method 3 failed: {e}")
        
        print("\n🎯 Testing completed!")
        
    except ImportError as e:
        print(f"❌ Import failed: {e}")
    except Exception as e:
        print(f"❌ Test failed: {e}")

if __name__ == "__main__":
    test_unitree_sdk_reading() 