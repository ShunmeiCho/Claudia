#!/usr/bin/env python3
# Simple Unitree connection test
import sys
import time
import os

# Test network connectivity
def test_ping():
    print("🔍 Testing network connectivity...")
    result = os.system("ping -c 1 192.168.123.161 > /dev/null 2>&1")
    if result == 0:
        print("✅ Network connection OK")
        return True
    else:
        print("❌ Network connection failed")
        return False

# Test SDK import
def test_import():
    print("🔍 Testing Unitree SDK import...")
    try:
        _project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
        sys.path.insert(0, os.path.join(_project_root, 'src'))
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
        print("✅ SDK import successful")
        return True
    except Exception as e:
        print(f"❌ SDK import failed: {e}")
        return False

# Test environment
def test_env():
    print("🔍 Testing environment...")
    rmw = os.environ.get('RMW_IMPLEMENTATION', '')
    dds_uri = os.environ.get('CYCLONEDX_URI', '')
    print(f"RMW_IMPLEMENTATION: {rmw}")
    print(f"CYCLONEDX_URI configured: {'✅' if dds_uri else '❌'}")
    return True

if __name__ == "__main__":
    print("🧪 Quick Unitree Connection Test")
    print("="*40)
    test_ping()
    test_import() 
    test_env()
    print("="*40)
