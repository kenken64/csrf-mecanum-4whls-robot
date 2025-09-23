#!/usr/bin/env python3
"""
Test script for ELRS Robot serial data parsing
"""

import re

def parse_serial_data(line):
    """Parse a line of serial data from Arduino"""
    try:
        # Extract basic values
        link_match = re.search(r'Link:\s*(UP|DOWN)', line)
        lock_match = re.search(r'Lock:\s*(ON|OFF)', line)
        dir_match = re.search(r'Dir:\s*([^|]+)', line)
        speed_match = re.search(r'Speed:\s*(\d+)', line)
        battery_match = re.search(r'Batt:([\d.]+)V', line)

        # Convert to numeric values
        link_status = 1 if link_match and link_match.group(1) == 'UP' else 0
        lock_status = 1 if lock_match and lock_match.group(1) == 'ON' else 0
        direction = dir_match.group(1).strip() if dir_match else 'UNKNOWN'
        speed = int(speed_match.group(1)) if speed_match else 0
        battery = float(battery_match.group(1)) if battery_match else 0.0

        return {
            'link_status': link_status,
            'lock_status': lock_status,
            'direction': direction,
            'speed': speed,
            'battery': battery
        }

    except Exception as e:
        print(f"Error parsing line: {e}")
        return None

# Test with sample data
test_lines = [
    "Link: UP | Lock: OFF | Dir: FORWARD LEFT | Speed: 180 | Batt:7.4V | CH1:1500 CH2:1750 CH3:1650 CH4:1650 CH8:2000 | FilteredCH2:1523",
    "Link: DOWN | Lock: ON | Dir: STOPPED | Speed: 60 | Batt:6.8V | CH1:1500 CH2:1500 CH3:1500 CH4:1500 CH8:1000 | FilteredCH2:1500",
    "Link: UP | Lock: OFF | Dir: STRAFE RIGHT | Speed: 120 | Batt:7.2V | CH1:1800 CH2:1600 CH3:1500 CH4:1500 CH8:2000 | FilteredCH2:1550"
]

print("Testing serial data parsing:")
print("=" * 50)

for i, line in enumerate(test_lines, 1):
    print(f"\nTest {i}:")
    print(f"Input: {line}")
    result = parse_serial_data(line)
    if result:
        print(f"Parsed: Link={result['link_status']}, Lock={result['lock_status']}, Dir='{result['direction']}', Speed={result['speed']}, Batt={result['battery']}V")
    else:
        print("Failed to parse")

print("\n" + "=" * 50)
print("Parsing test completed!")