#!/usr/bin/env python3
# Test different LowState_ initialization approaches
import sys
sys.path.insert(0, '/home/m1ng/claudia/src')

def test_lowstate_approaches():
    """Test various LowState_ initialization methods"""
    print("🧪 Testing LowState_ initialization approaches...")
    
    try:
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
        
        print("✅ Imports successful")
        
        # Approach 1: Direct creation with default values
        print("\n1. Testing direct LowState_() creation...")
        try:
            msg = LowState_()
            print("✅ Direct creation successful!")
            return True
        except TypeError as e:
            print(f"❌ Direct creation failed: {e}")
            print("   This means LowState_ requires initialization parameters")
        
        # Approach 2: Try creating subscriber first, then message
        print("\n2. Testing subscriber-first approach...")
        try:
            subscriber = ChannelSubscriber("rt/lowstate", LowState_)
            print("✅ Subscriber created successfully")
            
            # Now try to use the subscriber's message type
            print("   Attempting to read without message buffer...")
            result = subscriber.Read()  # Try without message parameter
            print(f"   Read result: {result}")
            
        except Exception as e:
            print(f"   Subscriber approach failed: {e}")
        
        # Approach 3: Check if there's a factory method
        print("\n3. Testing factory methods...")
        try:
            # Look for alternative message creation methods
            if hasattr(LowState_, 'create') or hasattr(LowState_, 'default'):
                print("   Found factory methods")
            else:
                print("   No obvious factory methods found")
                
        except Exception as e:
            print(f"   Factory method check failed: {e}")
        
        return False
        
    except Exception as e:
        print(f"❌ Critical import/setup failure: {e}")
        return False

if __name__ == "__main__":
    test_lowstate_approaches()
