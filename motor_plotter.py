#!/usr/bin/env python3
"""
ELRS Robot Control - Real-time PWM Motor Plotter
Visualizes motor control signals, speed, battery voltage, and ELRS status
"""

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import re
import time
import sys

# Configuration
SERIAL_PORT = '/dev/ttyACM0'  # Adjust for your system (e.g., 'COM3' on Windows)
BAUD_RATE = 115200
MAX_DATA_POINTS = 200  # Number of points to keep in history (increased for better visualization)

class MotorPlotter:
    def __init__(self, demo_mode=False):
        # Data storage
        self.time_data = deque(maxlen=MAX_DATA_POINTS)
        self.left_motor_data = deque(maxlen=MAX_DATA_POINTS)
        self.right_motor_data = deque(maxlen=MAX_DATA_POINTS)
        self.speed_data = deque(maxlen=MAX_DATA_POINTS)
        self.battery_data = deque(maxlen=MAX_DATA_POINTS)
        self.link_status_data = deque(maxlen=MAX_DATA_POINTS)
        self.lock_status_data = deque(maxlen=MAX_DATA_POINTS)

        # Direction tracking
        self.direction_data = deque(maxlen=MAX_DATA_POINTS)

        # Start time
        self.start_time = time.time()

        # Demo mode flag
        self.demo_mode = demo_mode

        # Setup serial connection (only if not in demo mode)
        if not self.demo_mode:
            self.setup_serial()
        else:
            self.ser = None

        # Setup matplotlib
        self.setup_plots()

    def setup_serial(self):
        """Setup serial connection"""
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
        except serial.SerialException as e:
            print(f"Error opening serial port {SERIAL_PORT}: {e}")
            print("Available ports:")
            import serial.tools.list_ports
            ports = list(serial.tools.list_ports.comports())
            for port in ports:
                print(f"  {port.device}: {port.description}")
            raise

    def setup_plots(self):
        """Setup matplotlib plots and subplots"""
        self.fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.suptitle('ELRS Robot Control - Real-time PWM Motor Plotter', fontsize=14)

        # Motor PWM signals
        self.line_left, = ax1.plot([], [], 'b-', label='Left Motor PWM', linewidth=2)
        self.line_right, = ax1.plot([], [], 'r-', label='Right Motor PWM', linewidth=2)
        ax1.set_title('Motor PWM Signals')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('PWM Value (0-255)')
        ax1.set_ylim(0, 280)
        ax1.grid(True)
        ax1.legend()

        # Speed and Battery
        self.line_speed, = ax2.plot([], [], 'g-', label='Motor Speed', linewidth=2)
        self.line_battery, = ax2.plot([], [], 'orange', label='Battery Voltage', linewidth=2)
        ax2.set_title('Speed & Battery Voltage')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Speed (60-255) / Voltage (V)')
        ax2.set_ylim(0, 15)
        ax2.grid(True)
        ax2.legend()

        # Link and Lock Status
        self.line_link, = ax3.plot([], [], 'purple', label='ELRS Link (1=UP, 0=DOWN)', linewidth=2)
        self.line_lock, = ax3.plot([], [], 'brown', label='Motor Lock (1=ON, 0=OFF)', linewidth=2)
        ax3.set_title('ELRS Link & Motor Lock Status')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Status (0/1)')
        ax3.set_ylim(-0.1, 1.1)
        ax3.grid(True)
        ax3.legend()

        # Direction indicator (text)
        ax4.axis('off')
        self.direction_text = ax4.text(0.1, 0.8, 'Direction: STOPPED',
                                     fontsize=12, verticalalignment='top',
                                     bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue"))
        self.status_text = ax4.text(0.1, 0.6, 'Status: Initializing...',
                                   fontsize=10, verticalalignment='top',
                                   bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgray"))

        plt.tight_layout()

    def parse_serial_data(self, line):
        """Parse a line of serial data from Arduino"""
        try:
            # Example line: "Link: UP | Lock: OFF | Dir: FORWARD LEFT | Speed: 180 | Batt:7.4V | CH1:1500 CH2:1750 CH3:1650 CH4:1650 CH8:2000 | FilteredCH2:1523"

            # Extract basic values
            link_match = re.search(r'Link:\s*(UP|DOWN)', line)
            lock_match = re.search(r'Lock:\s*(ON|OFF)', line)
            dir_match = re.search(r'Dir:\s*([^|]+)', line)
            speed_match = re.search(r'Speed:\s*(\d+)', line)
            battery_match = re.search(r'Batt:([\d.]+)V', line)

            # Convert to numeric values
            current_time = time.time() - self.start_time

            link_status = 1 if link_match and link_match.group(1) == 'UP' else 0
            lock_status = 1 if lock_match and lock_match.group(1) == 'ON' else 0
            direction = dir_match.group(1).strip() if dir_match else 'UNKNOWN'
            speed = int(speed_match.group(1)) if speed_match else 0
            battery = float(battery_match.group(1)) if battery_match else 0.0

            # For now, we'll estimate left/right motor values based on direction and speed
            # This is a simplified estimation - you could modify Arduino to send actual PWM values
            left_motor, right_motor = self.estimate_motor_pwms(direction, speed)

            return {
                'time': current_time,
                'left_motor': left_motor,
                'right_motor': right_motor,
                'speed': speed,
                'battery': battery,
                'link_status': link_status,
                'lock_status': lock_status,
                'direction': direction
            }

        except Exception as e:
            print(f"Error parsing line: {e}")
            return None

    def estimate_motor_pwms(self, direction, speed):
        """Estimate left and right motor PWM values based on direction and speed"""
        # This is a simplified estimation based on the Arduino code logic
        left_pwm = 0
        right_pwm = 0

        if 'FORWARD' in direction:
            if 'LEFT' in direction:
                left_pwm = speed
                right_pwm = int(speed * 0.5)  # Differential steering
            elif 'RIGHT' in direction:
                left_pwm = int(speed * 0.5)
                right_pwm = speed
            else:
                left_pwm = right_pwm = speed
        elif 'BACKWARD' in direction:
            if 'LEFT' in direction:
                left_pwm = -speed  # Negative for backward
                right_pwm = int(-speed * 0.5)
            elif 'RIGHT' in direction:
                left_pwm = int(-speed * 0.5)
                right_pwm = -speed
            else:
                left_pwm = right_pwm = -speed
        elif 'STRAFE' in direction or 'DRIFT' in direction:
            if 'LEFT' in direction:
                left_pwm = int(speed * 0.3)  # Slower for strafe
                right_pwm = speed
            elif 'RIGHT' in direction:
                left_pwm = speed
                right_pwm = int(speed * 0.3)
        elif 'TURN' in direction:
            if 'LEFT' in direction:
                left_pwm = int(speed * 0.3)
                right_pwm = speed
            elif 'RIGHT' in direction:
                left_pwm = speed
                right_pwm = int(speed * 0.3)

        return left_pwm, right_pwm

    def update_plot(self, frame):
        """Update the plot with new data"""
        try:
            # Read serial data
            if self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    data = self.parse_serial_data(line)
                    if data:
                        # Add data to deques
                        self.time_data.append(data['time'])
                        self.left_motor_data.append(data['left_motor'])
                        self.right_motor_data.append(data['right_motor'])
                        self.speed_data.append(data['speed'])
                        self.battery_data.append(data['battery'])
                        self.link_status_data.append(data['link_status'])
                        self.lock_status_data.append(data['lock_status'])
                        self.direction_data.append(data['direction'])

                        # Update plots
                        self.line_left.set_data(list(self.time_data), list(self.left_motor_data))
                        self.line_right.set_data(list(self.time_data), list(self.right_motor_data))
                        self.line_speed.set_data(list(self.time_data), list(self.speed_data))
                        self.line_battery.set_data(list(self.time_data), list(self.battery_data))
                        self.line_link.set_data(list(self.time_data), list(self.link_status_data))
                        self.line_lock.set_data(list(self.time_data), list(self.lock_status_data))

                        # Update direction text
                        current_dir = data['direction'] if self.direction_data else 'UNKNOWN'
                        self.direction_text.set_text(f'Direction: {current_dir}')

                        # Update status text
                        link_status = "UP" if data['link_status'] else "DOWN"
                        lock_status = "ON" if data['lock_status'] else "OFF"
                        self.status_text.set_text(f'ELRS: {link_status} | Lock: {lock_status} | Batt: {data["battery"]:.1f}V')

                        # Adjust x-axis limits
                        if self.time_data:
                            self.line_left.axes.set_xlim(max(0, self.time_data[-1] - 30), self.time_data[-1] + 1)
                            self.line_speed.axes.set_xlim(max(0, self.time_data[-1] - 30), self.time_data[-1] + 1)
                            self.line_link.axes.set_xlim(max(0, self.time_data[-1] - 30), self.time_data[-1] + 1)

        except Exception as e:
            print(f"Error updating plot: {e}")

        return self.line_left, self.line_right, self.line_speed, self.line_battery, self.line_link, self.line_lock

    def run_demo(self):
        """Run in demo mode with simulated data"""
        print("Demo mode: Generating simulated motor data...")
        print("Press Ctrl+C to stop")

        try:
            # Generate simulated data
            def demo_update(frame):
                current_time = time.time() - self.start_time

                # Simulate realistic motor behavior
                speed = 120 + 60 * abs((current_time % 10) - 5) / 5  # Vary speed 120-180
                battery = 7.2 + 0.5 * ((current_time % 20) - 10) / 10  # Vary battery 6.7-7.7V

                # Simulate different directions
                directions = ["FORWARD", "FORWARD LEFT", "FORWARD RIGHT", "BACKWARD",
                            "BACKWARD LEFT", "BACKWARD RIGHT", "STRAFE LEFT", "STRAFE RIGHT", "STOPPED"]
                direction = directions[int(current_time / 2) % len(directions)]

                # Estimate motor PWMs
                left_motor, right_motor = self.estimate_motor_pwms(direction, speed)

                # Add data
                self.time_data.append(current_time)
                self.left_motor_data.append(left_motor)
                self.right_motor_data.append(right_motor)
                self.speed_data.append(speed)
                self.battery_data.append(battery)
                self.link_status_data.append(1)  # Always UP in demo
                self.lock_status_data.append(0)  # Always OFF in demo
                self.direction_data.append(direction)

                # Update plots
                self.line_left.set_data(list(self.time_data), list(self.left_motor_data))
                self.line_right.set_data(list(self.time_data), list(self.right_motor_data))
                self.line_speed.set_data(list(self.time_data), list(self.speed_data))
                self.line_battery.set_data(list(self.time_data), list(self.battery_data))
                self.line_link.set_data(list(self.time_data), list(self.link_status_data))
                self.line_lock.set_data(list(self.time_data), list(self.lock_status_data))

                # Update text
                self.direction_text.set_text(f'Direction: {direction}')
                self.status_text.set_text(f'ELRS: UP | Lock: OFF | Batt: {battery:.1f}V (DEMO)')

                # Adjust x-axis
                if self.time_data:
                    xlim = max(0, current_time - 30)
                    self.line_left.axes.set_xlim(xlim, current_time + 1)
                    self.line_speed.axes.set_xlim(xlim, current_time + 1)
                    self.line_link.axes.set_xlim(xlim, current_time + 1)

                return self.line_left, self.line_right, self.line_speed, self.line_battery, self.line_link, self.line_lock

            ani = animation.FuncAnimation(self.fig, demo_update, interval=200, blit=True, cache_frame_data=False)
            plt.show()

        except KeyboardInterrupt:
            print("\nStopping demo...")
        finally:
            print("Demo completed")

    def run(self):
        """Start the plotting animation with live serial data"""
        print("Starting real-time motor plotter...")
        print("Press Ctrl+C to stop")

        try:
            ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100, blit=True, cache_frame_data=False)
            plt.show()
        except KeyboardInterrupt:
            print("\nStopping plotter...")
        finally:
            if self.ser:
                self.ser.close()
                print("Serial connection closed")

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='ELRS Robot Control - Real-time PWM Motor Plotter')
    parser.add_argument('--port', default=SERIAL_PORT, help=f'Serial port (default: {SERIAL_PORT})')
    parser.add_argument('--demo', action='store_true', help='Run in demo mode with simulated data')

    args = parser.parse_args()

    if args.demo:
        print("Running in DEMO mode with simulated data...")
        plotter = MotorPlotter(demo_mode=True)
        plotter.run_demo()
    else:
        plotter = MotorPlotter(demo_mode=False)
        plotter.run()