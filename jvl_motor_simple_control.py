import serial
import struct
import time
from typing import Optional, List, Dict

class JVLMotorSimpleController:
    """
    Simplified Python controller for JVL Stepper Motors
    Supports simultaneous and individual motor control without velocity/torque monitoring
    """

    def __init__(self, port: str, baudrate: int = 19200, timeout: float = 2.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_connection = None

        # Default position limits
        self.FULLY_OPEN = 7000000
        self.FULLY_CLOSED = -2000000

        # Motor IDs to control
        self.motor_ids = [1, 2, 3]

    def connect(self):
        """Establish serial connection"""
        try:
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_EVEN,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            print(f"Connected to {self.port} at {self.baudrate} baud")
            # Clear any existing data
            self.serial_connection.reset_input_buffer()
            self.serial_connection.reset_output_buffer()
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    def disconnect(self):
        """Close serial connection"""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            print("Disconnected")

    def calculate_crc16(self, data: bytes) -> int:
        """Calculate CRC16 for Modbus RTU"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def send_command(self, frame: bytes, expected_response_length: int = 8, show_debug: bool = False) -> Optional[bytes]:
        """Send Modbus command and receive response"""
        if not self.serial_connection or not self.serial_connection.is_open:
            print("Error: No active serial connection")
            return None

        try:
            # Clear buffers
            self.serial_connection.reset_input_buffer()

            # Send command
            if show_debug:
                print(f"Sending: {' '.join(f'{b:02X}' for b in frame)}")
            self.serial_connection.write(frame)

            # Wait for response
            time.sleep(0.1)

            # Read response
            response = self.serial_connection.read(expected_response_length)

            if len(response) == 0:
                print("Error: No response received")
                return None

            if show_debug:
                print(f"Received: {' '.join(f'{b:02X}' for b in response)}")

            if len(response) < 5:
                print(f"Error: Response too short ({len(response)} bytes)")
                return None

            # Verify CRC
            received_crc = struct.unpack('<H', response[-2:])[0]
            calculated_crc = self.calculate_crc16(response[:-2])

            if received_crc != calculated_crc:
                print(f"Error: CRC mismatch. Expected: {calculated_crc:04X}, Got: {received_crc:04X}")
                return None

            return response[:-2]  # Return without CRC

        except Exception as e:
            print(f"Communication error: {e}")
            return None

    def write_position(self, motor_id: int, position: int, show_debug: bool = False) -> bool:
        """
        Write position to P_SOLL register (register 3)

        Args:
            motor_id: Motor ID (1-247)
            position: Target position (32-bit signed integer)
            show_debug: Show debug information
        """
        # Register 3 (P_SOLL) -> Modbus address 6 (3 * 2)
        modbus_address = 6

        # Convert position to 32-bit signed integer, then split into words
        if position < 0:
            position = position & 0xFFFFFFFF  # Convert to unsigned representation

        low_word = position & 0xFFFF
        high_word = (position >> 16) & 0xFFFF

        # Build Modbus Write Multiple Registers command (Function 16)
        frame = struct.pack('>BBHHBHH',
                           motor_id,        # Slave ID
                           0x10,            # Function 16 (Write Multiple Registers)
                           modbus_address,  # Starting address
                           2,               # Number of registers (2 for 32-bit)
                           4,               # Byte count
                           low_word,        # Low word first
                           high_word)       # High word second

        # Add CRC
        crc = self.calculate_crc16(frame)
        frame += struct.pack('<H', crc)

        # Send command
        response = self.send_command(frame, 8, show_debug)

        if response is None:
            return False

        # Validate response
        if len(response) >= 6:
            resp_slave_id = response[0]
            resp_function = response[1]
            resp_addr = struct.unpack('>H', response[2:4])[0]
            resp_count = struct.unpack('>H', response[4:6])[0]

            if (resp_slave_id == motor_id and
                resp_function == 0x10 and
                resp_addr == modbus_address and
                resp_count == 2):
                if show_debug:
                    print(f"Successfully wrote position {position} to motor {motor_id}")
                return True
            else:
                print(f"Invalid response: slave={resp_slave_id}, func={resp_function}, addr={resp_addr}, count={resp_count}")

        return False

    def read_position(self, motor_id: int, show_debug: bool = False) -> Optional[int]:
        """Read current position from P_IST register (register 10)"""
        # Register 10 (P_IST) -> Modbus address 20 (10 * 2)
        modbus_address = 20

        # Build Modbus Read Holding Registers command (Function 3)
        frame = struct.pack('>BBHH',
                           motor_id,        # Slave ID
                           0x03,            # Function 3 (Read Holding Registers)
                           modbus_address,  # Starting address
                           2)               # Number of registers (2 for 32-bit)

        # Add CRC
        crc = self.calculate_crc16(frame)
        frame += struct.pack('<H', crc)

        # Send command
        response = self.send_command(frame, 9, show_debug)

        if response is None:
            return None

        # Validate response
        if len(response) >= 7:
            resp_slave_id = response[0]
            resp_function = response[1]
            byte_count = response[2]

            if (resp_slave_id == motor_id and
                resp_function == 0x03 and
                byte_count == 4):

                # Extract 32-bit value: Low Word, High Word
                low_word, high_word = struct.unpack('>HH', response[3:7])
                value = (high_word << 16) | low_word

                # Convert back to signed if necessary
                if value > 0x7FFFFFFF:
                    value = value - 0x100000000

                if show_debug:
                    print(f"Current position of motor {motor_id}: {value}")
                return value
            else:
                print(f"Invalid read response: slave={resp_slave_id}, func={resp_function}, bytes={byte_count}")

        return None

    # ==================== INDIVIDUAL MOTOR CONTROL ====================

    def move_motor(self, motor_id: int, position: int, show_status: bool = True) -> bool:
        """
        Move a single motor to specified position

        Args:
            motor_id: Motor ID (1-247)
            position: Target position
            show_status: Show status messages

        Returns:
            True if command sent successfully
        """
        if show_status:
            print(f"Moving motor {motor_id} to position {position}...")

        success = self.write_position(motor_id, position)

        if success and show_status:
            print(f"Motor {motor_id} command sent successfully")
        elif not success:
            print(f"Failed to send command to motor {motor_id}")

        return success

    def open_motor(self, motor_id: int, show_status: bool = True) -> bool:
        """Open a single motor to fully open position"""
        if show_status:
            print(f"Opening motor {motor_id} to position {self.FULLY_OPEN}...")
        return self.move_motor(motor_id, self.FULLY_OPEN, show_status)

    def close_motor(self, motor_id: int, show_status: bool = True) -> bool:
        """Close a single motor to fully closed position"""
        if show_status:
            print(f"Closing motor {motor_id} to position {self.FULLY_CLOSED}...")
        return self.move_motor(motor_id, self.FULLY_CLOSED, show_status)

    def get_motor_position(self, motor_id: int, show_status: bool = True) -> Optional[int]:
        """
        Get current position of a single motor

        Args:
            motor_id: Motor ID
            show_status: Show status messages

        Returns:
            Current position or None if failed
        """
        position = self.read_position(motor_id)

        if show_status:
            if position is not None:
                print(f"Motor {motor_id} position: {position}")
            else:
                print(f"Failed to read motor {motor_id} position")

        return position

    # ==================== SIMULTANEOUS MOTOR CONTROL ====================

    def move_all_motors(self, position: int, show_status: bool = True) -> Dict[int, bool]:
        """
        Move all motors to the same position simultaneously

        Args:
            position: Target position for all motors
            show_status: Show status messages

        Returns:
            Dictionary of {motor_id: success_status}
        """
        if show_status:
            print(f"\nMoving all motors to position {position}...")

        results = {}
        for motor_id in self.motor_ids:
            success = self.write_position(motor_id, position)
            results[motor_id] = success

            if show_status:
                status = "OK" if success else "FAILED"
                print(f"  Motor {motor_id}: {status}")

        return results

    def move_motors_individually(self, positions: Dict[int, int], show_status: bool = True) -> Dict[int, bool]:
        """
        Move multiple motors to different positions simultaneously

        Args:
            positions: Dictionary of {motor_id: target_position}
            show_status: Show status messages

        Returns:
            Dictionary of {motor_id: success_status}
        """
        if show_status:
            print(f"\nMoving motors to individual positions...")
            for motor_id, pos in positions.items():
                print(f"  Motor {motor_id} -> {pos}")

        results = {}
        for motor_id, position in positions.items():
            success = self.write_position(motor_id, position)
            results[motor_id] = success

            if show_status:
                status = "OK" if success else "FAILED"
                print(f"  Motor {motor_id}: {status}")

        return results

    def open_all_motors(self, show_status: bool = True) -> Dict[int, bool]:
        """Open all motors to fully open position"""
        if show_status:
            print(f"\nOpening all motors to position {self.FULLY_OPEN}...")
        return self.move_all_motors(self.FULLY_OPEN, show_status)

    def close_all_motors(self, show_status: bool = True) -> Dict[int, bool]:
        """Close all motors to fully closed position"""
        if show_status:
            print(f"\nClosing all motors to position {self.FULLY_CLOSED}...")
        return self.move_all_motors(self.FULLY_CLOSED, show_status)

    def get_all_positions(self, show_status: bool = True) -> Dict[int, Optional[int]]:
        """
        Read positions of all motors

        Args:
            show_status: Show status messages

        Returns:
            Dictionary of {motor_id: position}
        """
        if show_status:
            print(f"\nReading all motor positions...")

        positions = {}
        for motor_id in self.motor_ids:
            position = self.read_position(motor_id)
            positions[motor_id] = position

            if show_status:
                if position is not None:
                    print(f"  Motor {motor_id}: {position}")
                else:
                    print(f"  Motor {motor_id}: FAILED")

        return positions

    def stop_all_motors(self, show_status: bool = True) -> Dict[int, bool]:
        """
        Stop all motors at their current positions

        Args:
            show_status: Show status messages

        Returns:
            Dictionary of {motor_id: success_status}
        """
        if show_status:
            print(f"\nStopping all motors at current positions...")

        # First read all current positions
        current_positions = self.get_all_positions(show_status=False)

        # Send stop commands (set position to current position)
        results = {}
        for motor_id, position in current_positions.items():
            if position is not None:
                success = self.write_position(motor_id, position)
                results[motor_id] = success
                if show_status:
                    status = "STOPPED" if success else "FAILED"
                    print(f"  Motor {motor_id}: {status} at {position}")
            else:
                results[motor_id] = False
                if show_status:
                    print(f"  Motor {motor_id}: FAILED (couldn't read position)")

        return results

    # ==================== CONFIGURATION ====================

    def set_position_limits(self, fully_open: int, fully_closed: int):
        """Set the position limits for open and closed positions"""
        self.FULLY_OPEN = fully_open
        self.FULLY_CLOSED = fully_closed
        print(f"Position limits updated: Open={self.FULLY_OPEN}, Closed={self.FULLY_CLOSED}")

    def set_motor_ids(self, motor_ids: List[int]):
        """Set which motor IDs to control"""
        self.motor_ids = motor_ids
        print(f"Motor IDs set to: {self.motor_ids}")


def main():
    """Simple motor control program"""
    # Configuration
    COM_PORT = "COM10"          # Your COM port
    MOTOR_IDS = [1, 2, 3]       # Motor IDs to control
    FULLY_OPEN = 7000000        # Fully open position
    FULLY_CLOSED = -2000000     # Fully closed position

    # Create controller
    controller = JVLMotorSimpleController(COM_PORT, baudrate=19200, timeout=2.0)

    # Set configuration
    controller.set_motor_ids(MOTOR_IDS)
    controller.set_position_limits(FULLY_OPEN, FULLY_CLOSED)

    # Connect
    if not controller.connect():
        print("Failed to connect")
        return

    try:
        # Main control loop
        while True:
            print("\n" + "="*60)
            print("JVL MOTOR SIMPLE CONTROLLER")
            print("="*60)
            print("\n--- INDIVIDUAL MOTOR CONTROL ---")
            print("1. Move single motor to position")
            print("2. Open single motor")
            print("3. Close single motor")
            print("4. Read single motor position")

            print("\n--- SIMULTANEOUS MOTOR CONTROL ---")
            print("5. Move all motors to same position")
            print("6. Move motors to different positions")
            print("7. Open all motors")
            print("8. Close all motors")
            print("9. Read all motor positions")
            print("10. Stop all motors at current positions")

            print("\n--- CONFIGURATION ---")
            print("11. Change position limits")
            print("12. Change motor IDs")

            print("\n0. Exit")
            print("="*60)

            choice = input("Enter your choice: ").strip()

            if choice == "1":
                # Move single motor
                try:
                    motor_id = int(input(f"Enter motor ID {controller.motor_ids}: "))
                    if motor_id not in controller.motor_ids:
                        print(f"Invalid motor ID. Available: {controller.motor_ids}")
                        continue
                    position = int(input("Enter target position: "))
                    controller.move_motor(motor_id, position)
                except ValueError:
                    print("Invalid input. Please enter numbers only.")

            elif choice == "2":
                # Open single motor
                try:
                    motor_id = int(input(f"Enter motor ID {controller.motor_ids}: "))
                    if motor_id not in controller.motor_ids:
                        print(f"Invalid motor ID. Available: {controller.motor_ids}")
                        continue
                    controller.open_motor(motor_id)
                except ValueError:
                    print("Invalid input. Please enter numbers only.")

            elif choice == "3":
                # Close single motor
                try:
                    motor_id = int(input(f"Enter motor ID {controller.motor_ids}: "))
                    if motor_id not in controller.motor_ids:
                        print(f"Invalid motor ID. Available: {controller.motor_ids}")
                        continue
                    controller.close_motor(motor_id)
                except ValueError:
                    print("Invalid input. Please enter numbers only.")

            elif choice == "4":
                # Read single motor position
                try:
                    motor_id = int(input(f"Enter motor ID {controller.motor_ids}: "))
                    if motor_id not in controller.motor_ids:
                        print(f"Invalid motor ID. Available: {controller.motor_ids}")
                        continue
                    controller.get_motor_position(motor_id)
                except ValueError:
                    print("Invalid input. Please enter numbers only.")

            elif choice == "5":
                # Move all motors to same position
                try:
                    position = int(input("Enter target position for all motors: "))
                    controller.move_all_motors(position)
                except ValueError:
                    print("Invalid input. Please enter numbers only.")

            elif choice == "6":
                # Move motors to different positions
                try:
                    positions = {}
                    print("Enter positions for each motor:")
                    for motor_id in controller.motor_ids:
                        pos = int(input(f"  Motor {motor_id} position: "))
                        positions[motor_id] = pos
                    controller.move_motors_individually(positions)
                except ValueError:
                    print("Invalid input. Please enter numbers only.")

            elif choice == "7":
                # Open all motors
                controller.open_all_motors()

            elif choice == "8":
                # Close all motors
                controller.close_all_motors()

            elif choice == "9":
                # Read all positions
                controller.get_all_positions()

            elif choice == "10":
                # Stop all motors
                controller.stop_all_motors()

            elif choice == "11":
                # Change position limits
                try:
                    print(f"Current limits: Open={controller.FULLY_OPEN}, Closed={controller.FULLY_CLOSED}")
                    new_open = int(input("Enter new FULLY OPEN position: "))
                    new_closed = int(input("Enter new FULLY CLOSED position: "))
                    controller.set_position_limits(new_open, new_closed)
                except ValueError:
                    print("Invalid input. Please enter numbers only.")

            elif choice == "12":
                # Change motor IDs
                try:
                    print(f"Current motor IDs: {controller.motor_ids}")
                    ids_str = input("Enter motor IDs separated by commas (e.g., 1,2,3): ")
                    new_ids = [int(x.strip()) for x in ids_str.split(",")]
                    controller.set_motor_ids(new_ids)
                except ValueError:
                    print("Invalid input. Please enter numbers separated by commas.")

            elif choice == "0":
                print("Goodbye!")
                break

            else:
                print("Invalid choice. Please try again.")

    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        controller.disconnect()


if __name__ == "__main__":
    main()
