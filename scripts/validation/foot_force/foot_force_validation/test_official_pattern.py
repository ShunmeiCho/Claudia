#!/usr/bin/env python3
# Test official Unitree SDK pattern for reading data
import sys
sys.path.insert(0, '/home/m1ng/claudia/src')

def test_official_read_pattern():
    """Test the official way to read data from Unitree robot"""
    print("🧪 Testing Official Unitree SDK Read Pattern...")
    
    try:
        from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
        
        # Step 1: Initialize DDS channel factory
        print("1. Initializing DDS channel factory...")
        ChannelFactoryInitialize(0, "eth0")  # domain_id=0, interface="eth0"
        print("✅ DDS factory initialized")
        
        # Step 2: Create subscriber 
        print("2. Creating subscriber...")
        subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        print("✅ Subscriber created")
        
        # Step 3: Try different read approaches
        print("3. Testing data reading approaches...")
        
        # Approach A: Check if Read() has different signatures
        import inspect
        read_signature = inspect.signature(subscriber.Read)
        print(f"   Read method signature: {read_signature}")
        
        # Approach B: Try reading with timeout
        print("   Attempting read with various parameters...")
        try:
            # Some SDK versions might support reading without pre-created message
            result = subscriber.Read()
            print(f"   Read() without parameters result: {result}")
        except Exception as e:
            print(f"   Read() without parameters failed: {e}")
        
        # Approach C: Try with timeout parameter
        try:
            result = subscriber.Read(1000)  # 1 second timeout in milliseconds
            print(f"   Read(timeout) result: {result}")
        except Exception as e:
            print(f"   Read(timeout) failed: {e}")
            
        # Approach D: Check available methods
        print("   Available subscriber methods:")
        methods = [method for method in dir(subscriber) if not method.startswith('_')]
        for method in methods:
            print(f"     - {method}")
            
        return True
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        return False

if __name__ == "__main__":
    test_official_read_pattern()
