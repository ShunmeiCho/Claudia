#!/usr/bin/env python3
"""
Claudia Robot Project Test Runner

Provides a unified test runner interface with support for different test types and configuration options.
"""

import sys
import os
import argparse
import subprocess
import time
from pathlib import Path
from typing import List, Optional

# Add project root directory to Python path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

class TestRunner:
    """Test runner"""

    def __init__(self):
        self.test_dir = Path(__file__).parent
        self.project_root = self.test_dir.parent
        self.results = {}

    def run_hardware_tests(self, verbose: bool = False) -> bool:
        """Run hardware tests (auto-discovers all test_*.py files)"""
        print("Running hardware tests...")

        hardware_test_files = sorted((self.test_dir / "hardware").glob("test_*.py"))
        if not hardware_test_files:
            print("  No hardware test files found")
            return True

        success = True
        for test_path in hardware_test_files:
            print(f"  Running {test_path.name}...")
            result = self._run_single_test(test_path, verbose)
            self.results[f"hardware/{test_path.name}"] = result
            if not result:
                success = False
                print(f"    FAILED: {test_path.name}")
            else:
                print(f"    PASSED: {test_path.name}")

        return success

    def run_unit_tests(self, verbose: bool = False) -> bool:
        """Run unit tests"""
        print("Running unit tests...")

        unit_test_files = list((self.test_dir / "unit").glob("test_*.py"))
        if not unit_test_files:
            print("  No unit test files found")
            return True

        success = True
        for test_file in unit_test_files:
            print(f"  Running {test_file.name}...")
            result = self._run_single_test(test_file, verbose)
            self.results[f"unit/{test_file.name}"] = result
            if not result:
                success = False
                print(f"    FAILED: {test_file.name}")
            else:
                print(f"    PASSED: {test_file.name}")

        return success

    def run_integration_tests(self, verbose: bool = False) -> bool:
        """Run integration tests"""
        print("Running integration tests...")

        integration_test_files = list((self.test_dir / "integration").glob("test_*.py"))
        if not integration_test_files:
            print("  No integration test files found")
            return True

        success = True
        for test_file in integration_test_files:
            print(f"  Running {test_file.name}...")
            result = self._run_single_test(test_file, verbose)
            self.results[f"integration/{test_file.name}"] = result
            if not result:
                success = False
                print(f"    FAILED: {test_file.name}")
            else:
                print(f"    PASSED: {test_file.name}")

        return success

    def _run_single_test(self, test_path: Path, verbose: bool = False) -> bool:
        """Run a single test file"""
        try:
            cmd = [sys.executable, str(test_path)]
            # Propagate src/ in PYTHONPATH so subprocess can import claudia modules
            env = os.environ.copy()
            src_dir = str(self.project_root / "src")
            existing = env.get("PYTHONPATH", "")
            env["PYTHONPATH"] = src_dir + (":" + existing if existing else "")
            if verbose:
                result = subprocess.run(cmd, cwd=self.project_root,
                                      env=env, capture_output=False, text=True)
            else:
                result = subprocess.run(cmd, cwd=self.project_root,
                                      env=env, capture_output=True, text=True)

            return result.returncode == 0

        except Exception as e:
            print(f"    Error running {test_path.name}: {e}")
            return False

    def print_summary(self):
        """Print test results summary"""
        print("\n" + "="*60)
        print("Test Results Summary")
        print("="*60)

        total_tests = len(self.results)
        passed_tests = sum(1 for result in self.results.values() if result)
        failed_tests = total_tests - passed_tests

        print(f"Total tests: {total_tests}")
        print(f"Passed: {passed_tests}")
        print(f"Failed: {failed_tests}")

        if failed_tests > 0:
            print("\nFailed tests:")
            for test_name, result in self.results.items():
                if not result:
                    print(f"  - {test_name}")

        print("="*60)
        return failed_tests == 0

def main():
    """Main function"""
    parser = argparse.ArgumentParser(description="Claudia Robot Project Test Runner")
    parser.add_argument("--type", choices=["all", "unit", "integration", "hardware"],
                       default="all", help="Run a specific type of tests")
    parser.add_argument("-v", "--verbose", action="store_true",
                       help="Show detailed output")
    parser.add_argument("--debug", action="store_true",
                       help="Debug mode")

    args = parser.parse_args()

    print("Claudia Robot Project Test Runner")
    print(f"Start time: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"Test type: {args.type}")
    print("-" * 60)

    runner = TestRunner()
    overall_success = True

    try:
        if args.type in ["all", "unit"]:
            success = runner.run_unit_tests(args.verbose)
            overall_success = overall_success and success

        if args.type in ["all", "integration"]:
            success = runner.run_integration_tests(args.verbose)
            overall_success = overall_success and success

        if args.type in ["all", "hardware"]:
            print("\nWARNING: Hardware tests require a robot connection. Ensure:")
            print("   1. Go2 robot is powered on and connected")
            print("   2. Network configuration is correct")
            print("   3. CycloneDDS environment is set up")

            if args.debug or input("\nContinue running hardware tests? (y/N): ").lower() == 'y':
                success = runner.run_hardware_tests(args.verbose)
                overall_success = overall_success and success
            else:
                print("Skipping hardware tests")

    except KeyboardInterrupt:
        print("\n\nTests interrupted by user")
        return 1
    except Exception as e:
        print(f"\nTest run error: {e}")
        return 1

    # Print summary
    final_success = runner.print_summary()

    print(f"End time: {time.strftime('%Y-%m-%d %H:%M:%S')}")

    return 0 if final_success else 1

if __name__ == "__main__":
    exit(main())
