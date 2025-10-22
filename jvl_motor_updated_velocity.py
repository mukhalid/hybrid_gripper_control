import serial
import struct
import time
from typing import Optional, List

class JVLMotorController:
    """
    Python controller for JVL Stepper Mo2tors - Multi-Motor Position Control with Smart Grip
    Updated version with working stop mechanism for torque-based control
    """
    
    def __init__(self, port: str, baudrate: int = 19200, timeout: float = 2.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_connection = None
        
        # Default position limits - easily changeable
        self.FULLY_OPEN = 7000000
        self.FULLY_CLOSED = -2000000  # Updated to match your system
        
        # Motor IDs to control
        self.motor_ids = [1, 2, 3]
        
        # Smart grip settings - VELOCITY-BASED DETECTION
        self.GRIP_DETECTION_MODE = "VELOCITY"  # "VELOCITY" or "TORQUE"
        self.GRIP_VELOCITY_THRESHOLD = 500     # Consider stopped if velocity below this
        self.GRIP_STALL_COUNT = 5              # Number of low velocity readings to confirm grip
        self.GRIP_POSITION_TOLERANCE = 5000    # Position change tolerance during stall
        
        # Torque settings (backup/optional)
        self.GRIP_TORQUE_CHANGE_THRESHOLD = 8000   # Default threshold
        self.GRIP_DETECTION_ENABLED = True         # Can disable for testing
        self.MOTOR_SPECIFIC_THRESHOLDS = {         # Individual motor thresholds
            1: 15000,  # Motor 1
            2: 25000,  # Motor 2
            3: 20000   # Motor 3
        }
        
        # Motion control mode
        self.USE_INCREMENTAL_MOVEMENT = True  # Use incremental movements
        self.INCREMENT_SIZE = 7000000  # Large steps as you requested
        self.INCREMENT_DELAY = 0.05  # Small delay between increments
        
        # Position limits safety
        self.POSITION_MIN_LIMIT = -2100000  # Don't go below this
        self.POSITION_MAX_LIMIT = 7000000   # Don't go above this
        
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
    
    def write_velocity(self, motor_id: int, velocity: int, show_debug: bool = False) -> bool:
        """
        Write velocity to V_SOLL register (register 4)
        
        Args:
            motor_id: Motor ID (1-247)
            velocity: Target velocity (32-bit signed integer)
            show_debug: Show debug information
        """
        # Register 4 (V_SOLL) -> Modbus address 8 (4 * 2)
        modbus_address = 8
        
        # Convert velocity to 32-bit signed integer, then split into words
        if velocity < 0:
            velocity = velocity & 0xFFFFFFFF  # Convert to unsigned representation
        
        low_word = velocity & 0xFFFF
        high_word = (velocity >> 16) & 0xFFFF
        
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
                    print(f"Successfully wrote velocity {velocity} to motor {motor_id}")
                return True
            else:
                print(f"Invalid velocity response: slave={resp_slave_id}, func={resp_function}, addr={resp_addr}, count={resp_count}")
        
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
    
    def read_register_32bit(self, motor_id: int, register: int, register_name: str = "", show_debug: bool = False) -> Optional[int]:
        """
        Generic function to read a 32-bit register
        
        Args:
            motor_id: Motor ID (1-247)
            register: Register number
            register_name: Name for debug output
            show_debug: Show debug information
        
        Returns:
            Register value or None if failed
        """
        # Convert register number to Modbus address (multiply by 2 for 32-bit registers)
        modbus_address = register * 2
        
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
                
                if show_debug and register_name:
                    print(f"{register_name} of motor {motor_id}: {value}")
                return value
            else:
                print(f"Invalid read response: slave={resp_slave_id}, func={resp_function}, bytes={byte_count}")
        
        return None
    
    def read_position(self, motor_id: int, show_debug: bool = False) -> Optional[int]:
        """Read current position from P_IST register (register 10)"""
        return self.read_register_32bit(motor_id, 10, "Current position", show_debug)
    
    def read_velocity(self, motor_id: int, show_debug: bool = False) -> Optional[int]:
        """Read current velocity from V_IST register (register 11)"""
        return self.read_register_32bit(motor_id, 11, "Current velocity", show_debug)
    
    def read_torque(self, motor_id: int, show_debug: bool = False) -> Optional[int]:
        """Read current torque from T_IST register (register 12)"""
        return self.read_register_32bit(motor_id, 12, "Current torque", show_debug)
    
    def read_current(self, motor_id: int, show_debug: bool = False) -> Optional[int]:
        """Read current from I_IST register (register 13)"""
        return self.read_register_32bit(motor_id, 13, "Current", show_debug)
    
    def read_all_feedback(self, motor_id: int, show_output: bool = True) -> dict:
        """
        Read all feedback parameters for a motor
        
        Args:
            motor_id: Motor ID (1-247)
            show_output: Display the results
        
        Returns:
            Dictionary with all feedback values
        """
        feedback = {
            'position': self.read_position(motor_id),
            'velocity': self.read_velocity(motor_id),
            'torque': self.read_torque(motor_id),
            'current': self.read_current(motor_id)
        }
        
        if show_output:
            print(f"\n📊 Motor {motor_id} Feedback:")
            print(f"  Position: {feedback['position']:>10} units" if feedback['position'] is not None else "  Position: ERROR")
            print(f"  Velocity: {feedback['velocity']:>10} units/s" if feedback['velocity'] is not None else "  Velocity: ERROR")
            print(f"  Torque:   {feedback['torque']:>10} units" if feedback['torque'] is not None else "  Torque:   ERROR")
            print(f"  Current:  {feedback['current']:>10} units" if feedback['current'] is not None else "  Current:  ERROR")
        
        return feedback
    
    def set_position_limits(self, fully_open: int, fully_closed: int):
        """Set the position limits for open and closed positions"""
        self.FULLY_OPEN = fully_open
        self.FULLY_CLOSED = fully_closed
        print(f"Position limits updated: Open={self.FULLY_OPEN}, Closed={self.FULLY_CLOSED}")
    
    def set_grip_limits(self, torque_change_threshold: int):
        """Set the grip torque change threshold for safe gripping"""
        self.GRIP_TORQUE_CHANGE_THRESHOLD = torque_change_threshold
        print(f"Grip torque change threshold updated: {self.GRIP_TORQUE_CHANGE_THRESHOLD}")
    
    def set_motor_ids(self, motor_ids: List[int]):
        """Set which motor IDs to control"""
        self.motor_ids = motor_ids
        print(f"Motor IDs set to: {self.motor_ids}")
    
    def smart_grip_single_motor(self, motor_id: int, target_position: int = None, update_interval: float = 0.1) -> dict:
        """
        Smart grip control for single motor with incremental movement and torque monitoring
        
        Args:
            motor_id: Motor ID to control
            target_position: Target position (uses FULLY_CLOSED if None)
            update_interval: Monitoring frequency (reduced to 0.1 for faster response)
        
        Returns:
            Dictionary with grip results
        """
        if target_position is None:
            target_position = self.FULLY_CLOSED
            
        print(f"\n🤏 Smart Grip Motor {motor_id} to position {target_position}")
        print(f"Mode: {'INCREMENTAL' if self.USE_INCREMENTAL_MOVEMENT else 'DIRECT'}")
        print(f"Torque threshold: {self.MOTOR_SPECIFIC_THRESHOLDS.get(motor_id, self.GRIP_TORQUE_CHANGE_THRESHOLD)}")
        
        # Get current position
        current_pos = self.read_position(motor_id)
        if current_pos is None:
            return {"success": False, "reason": "Failed to read initial position", "final_position": None}
        
        print("-" * 90)
        print(f"{'Time':>6} | {'Position':>10} | {'Velocity':>10} | {'Torque':>10} | {'Current':>10} | {'Status':>15}")
        print("-" * 90)
        
        start_time = time.time()
        grip_detected = False
        final_position = current_pos
        stop_reason = "Unknown"
        baseline_torque = None
        
        # Calculate direction
        direction = 1 if target_position > current_pos else -1
        
        try:
            if self.USE_INCREMENTAL_MOVEMENT:
                # Incremental movement approach
                next_target = current_pos
                
                while True:
                    current_time = time.time() - start_time
                    
                    # Calculate next incremental target
                    if direction > 0:
                        next_target = min(next_target + self.INCREMENT_SIZE, target_position)
                    else:
                        next_target = max(next_target - self.INCREMENT_SIZE, target_position)
                    
                    # Send incremental move command
                    if not self.write_position(motor_id, next_target):
                        print(f"Failed to send position command to motor {motor_id}")
                        break
                    
                    # Monitor until this increment is reached or grip detected
                    increment_start_time = time.time()
                    
                    while True:
                        # Read feedback
                        position = self.read_position(motor_id)
                        velocity = self.read_velocity(motor_id)
                        torque = self.read_torque(motor_id)
                        current = self.read_current(motor_id)
                        
                        if position is None:
                            print(f"{current_time:6.1f} | {'ERROR':>10} | {'ERROR':>10} | {'ERROR':>10} | {'ERROR':>10} | {'COMM ERROR':>15}")
                            grip_detected = True
                            stop_reason = "Communication error"
                            break
                        
                        # Handle None values
                        velocity = velocity if velocity is not None else 0
                        torque = torque if torque is not None else 0
                        current = current if current is not None else 0
                        
                        final_position = position
                        status = "Moving"
                        
                        # Establish baseline torque
                        if baseline_torque is None:
                            baseline_torque = torque
                            status = "Baseline Set"
                        else:
                            # Check torque change
                            motor_threshold = self.MOTOR_SPECIFIC_THRESHOLDS.get(motor_id, self.GRIP_TORQUE_CHANGE_THRESHOLD)
                            torque_change = abs(torque - baseline_torque)
                            
                            if self.GRIP_DETECTION_ENABLED and torque_change > motor_threshold:
                                status = "GRIP DETECTED"
                                grip_detected = True
                                stop_reason = f"Torque change: {torque_change} > {motor_threshold}"
                        
                        # Check if increment reached
                        position_error = abs(position - next_target)
                        if position_error < 1000:
                            status = "Increment Done"
                            break
                        
                        # Check if final target reached
                        if abs(position - target_position) < 1000:
                            status = "TARGET REACHED"
                            grip_detected = True
                            stop_reason = "Target position reached"
                        
                        print(f"{current_time:6.1f} | {position:>10} | {velocity:>10} | {torque:>10} | {current:>10} | {status:>15}")
                        
                        if grip_detected:
                            break
                        
                        # Timeout for this increment
                        if time.time() - increment_start_time > 2.0:
                            break
                        
                        time.sleep(update_interval)
                        current_time = time.time() - start_time
                    
                    if grip_detected:
                        break
                    
                    # Check if final target reached
                    if next_target == target_position:
                        break
                    
                    # Overall timeout
                    if current_time > 15.0:
                        stop_reason = "Timeout reached"
                        break
                    
                    # Small delay between increments
                    time.sleep(self.INCREMENT_DELAY)
                    
            else:
                # Direct movement approach (original)
                if not self.write_position(motor_id, target_position):
                    return {"success": False, "reason": "Failed to send command", "final_position": None}
                
                while True:
                    current_time = time.time() - start_time
                    
                    # Read feedback
                    position = self.read_position(motor_id)
                    velocity = self.read_velocity(motor_id)
                    torque = self.read_torque(motor_id)
                    current = self.read_current(motor_id)
                    
                    if position is None:
                        print(f"{current_time:6.1f} | {'ERROR':>10} | {'ERROR':>10} | {'ERROR':>10} | {'ERROR':>10} | {'COMM ERROR':>15}")
                        break
                    
                    # Handle None values
                    velocity = velocity if velocity is not None else 0
                    torque = torque if torque is not None else 0
                    current = current if current is not None else 0
                    
                    final_position = position
                    status = "Moving"
                    
                    # Establish baseline torque
                    if baseline_torque is None:
                        baseline_torque = torque
                        status = "Baseline Set"
                    else:
                        # Check conditions
                        position_error = abs(position - target_position)
                        motor_threshold = self.MOTOR_SPECIFIC_THRESHOLDS.get(motor_id, self.GRIP_TORQUE_CHANGE_THRESHOLD)
                        torque_change = abs(torque - baseline_torque)
                        
                        if position_error < 1000:
                            status = "TARGET REACHED"
                            grip_detected = True
                            stop_reason = "Target position reached"
                        elif position_error < 10000:
                            status = "Approaching"
                        elif self.GRIP_DETECTION_ENABLED and torque_change > motor_threshold and position_error > 50000:
                            status = "GRIP DETECTED"
                            grip_detected = True
                            stop_reason = f"Object contact: torque change {torque_change}"
                    
                    print(f"{current_time:6.1f} | {position:>10} | {velocity:>10} | {torque:>10} | {current:>10} | {status:>15}")
                    
                    if grip_detected:
                        print(f"🤏 Grip detected! Reason: {stop_reason}")
                        print(f"🛑 STOPPING Motor {motor_id} at position {final_position}")
                        
                        # Multiple stop attempts
                        self.write_position(motor_id, final_position)
                        time.sleep(0.01)
                        self.write_position(motor_id, final_position)
                        break
                    
                    if current_time > 15.0:
                        stop_reason = "Timeout reached"
                        break
                    
                    time.sleep(update_interval)
                    
        except KeyboardInterrupt:
            print(f"\n⏹️  Smart grip stopped by user")
            stop_reason = "User interrupted"
            grip_detected = True
        
        print("-" * 90)
        
        # Ensure motor is stopped
        if grip_detected or stop_reason == "Timeout reached":
            print(f"🛑 Final stop command for motor {motor_id} at position {final_position}")
            self.write_position(motor_id, final_position)
            self.write_velocity(motor_id, 0)
        
        return {
            "success": grip_detected,
            "reason": stop_reason,
            "final_position": final_position,
            "motor_id": motor_id
        }
    
    def smart_grip_all_motors(self, update_interval: float = 0.05) -> dict:
        """
        Smart grip control using velocity-based stall detection
        Motors stop when they stall (velocity near zero while trying to move)
        
        Args:
            update_interval: Monitoring frequency
        
        Returns:
            Dictionary with results for each motor
        """
        print(f"\n🤏 Smart Grip All Motors to position {self.FULLY_CLOSED}")
        print(f"Mode: {self.GRIP_DETECTION_MODE} Detection")
        print(f"Velocity threshold: {self.GRIP_VELOCITY_THRESHOLD}, Stall count: {self.GRIP_STALL_COUNT}")
        
        # Get initial positions
        initial_positions = {}
        for motor_id in self.motor_ids:
            pos = self.read_position(motor_id)
            if pos is not None:
                initial_positions[motor_id] = pos
            else:
                print(f"❌ Failed to read initial position for motor {motor_id}")
        
        if not initial_positions:
            return {"success": False, "reason": "No motors responded", "results": {}}
        
        # Check if any motor is already at or past target
        for motor_id, pos in initial_positions.items():
            if pos <= self.FULLY_CLOSED:
                print(f"⚠️  Motor {motor_id} already at/past target position ({pos})")
        
        print("-" * 130)
        header = f"{'Time':>6} |"
        for motor_id in sorted(initial_positions.keys()):
            header += f" Motor {motor_id} (Pos/Vel/LastPos/Stall/Status) |"
        print(header)
        print("-" * 130)
        
        start_time = time.time()
        motor_states = {}
        results = {}
        
        # Initialize tracking for each motor
        for motor_id in initial_positions.keys():
            motor_states[motor_id] = {
                "active": True,
                "stopped": False,
                "position": initial_positions[motor_id],
                "last_position": initial_positions[motor_id],
                "stall_count": 0,
                "command_sent": False,
                "last_command_time": 0,
                "no_movement_count": 0
            }
        
        # Send initial movement commands
        for motor_id in initial_positions.keys():
            if motor_states[motor_id]["position"] > self.FULLY_CLOSED:
                self.write_position(motor_id, self.FULLY_CLOSED)
                motor_states[motor_id]["command_sent"] = True
                motor_states[motor_id]["last_command_time"] = time.time()
        
        try:
            while True:
                current_time = time.time() - start_time
                row = f"{current_time:6.1f} |"
                
                active_motors = 0
                
                for motor_id in sorted(initial_positions.keys()):
                    if motor_states[motor_id]["stopped"]:
                        row += f" {'STOPPED':>42} |"
                        continue
                    
                    # Read feedback
                    position = self.read_position(motor_id)
                    velocity = self.read_velocity(motor_id)
                    
                    if position is None:
                        row += f" {'ERROR':>42} |"
                        continue
                    
                    velocity = velocity if velocity is not None else 0
                    
                    # Update state
                    motor_states[motor_id]["position"] = position
                    
                    # Calculate position change
                    position_change = abs(position - motor_states[motor_id]["last_position"])
                    
                    # Determine status
                    status = "Moving"
                    stop_motor = False
                    stop_reason = ""
                    
                    # Check if at position limit (0 or negative)
                    if position <= 0:
                        status = "LIMIT"
                        stop_motor = True
                        stop_reason = f"Position limit reached ({position})"
                    
                    # Check if target reached
                    elif abs(position - self.FULLY_CLOSED) < 1000:
                        status = "TARGET"
                        stop_motor = True
                        stop_reason = "Target position reached"
                    
                    # Check if motor is stalled (velocity-based detection)
                    elif self.GRIP_DETECTION_MODE == "VELOCITY":
                        # Check if motor should be moving but isn't
                        time_since_command = time.time() - motor_states[motor_id]["last_command_time"]
                        
                        if motor_states[motor_id]["command_sent"] and time_since_command > 0.5:
                            # Motor should be moving by now
                            if abs(velocity) < self.GRIP_VELOCITY_THRESHOLD and position_change < self.GRIP_POSITION_TOLERANCE:
                                motor_states[motor_id]["stall_count"] += 1
                                status = f"Stall{motor_states[motor_id]['stall_count']}"
                                
                                if motor_states[motor_id]["stall_count"] >= self.GRIP_STALL_COUNT:
                                    status = "STALLED"
                                    stop_motor = True
                                    stop_reason = f"Motor stalled (vel={velocity}, pos_change={position_change})"
                            else:
                                motor_states[motor_id]["stall_count"] = 0
                                status = "Moving"
                        else:
                            status = "Starting"
                    
                    # Check for no movement at all
                    if position_change < 100:
                        motor_states[motor_id]["no_movement_count"] += 1
                        if motor_states[motor_id]["no_movement_count"] > 20:  # 1 second at 50ms intervals
                            status = "NO_MOVE"
                            stop_motor = True
                            stop_reason = "No movement detected"
                    else:
                        motor_states[motor_id]["no_movement_count"] = 0
                    
                    # Format display
                    motor_display = f" {position:>8}/{velocity:>6}/{motor_states[motor_id]['last_position']:>8}/{motor_states[motor_id]['stall_count']:>5}/{status:>10} |"
                    row += motor_display
                    
                    # Update last position
                    motor_states[motor_id]["last_position"] = position
                    
                    if stop_motor and motor_states[motor_id]["active"]:
                        # Stop this motor
                        motor_states[motor_id]["active"] = False
                        print(row)
                        print(f"🛑 Motor {motor_id} stopped: {stop_reason}")
                        
                        # Aggressive stop sequence
                        for i in range(10):
                            self.write_velocity(motor_id, 0)
                            time.sleep(0.005)
                            self.write_position(motor_id, position)
                            time.sleep(0.005)
                        
                        motor_states[motor_id]["stopped"] = True
                        results[motor_id] = {
                            "success": True,
                            "reason": stop_reason,
                            "final_position": position
                        }
                    
                    if motor_states[motor_id]["active"]:
                        active_motors += 1
                
                print(row)
                
                # Stop if all motors have stopped
                if active_motors == 0:
                    print("🤏 All motors have completed gripping!")
                    break
                
                # Safety timeout
                if current_time > 25.0:
                    print("⏰ Smart grip timeout reached")
                    for motor_id in initial_positions.keys():
                        if motor_states[motor_id]["active"]:
                            pos = motor_states[motor_id]["position"]
                            # Force stop
                            for _ in range(5):
                                self.write_velocity(motor_id, 0)
                                self.write_position(motor_id, pos)
                                time.sleep(0.01)
                            results[motor_id] = {
                                "success": False,
                                "reason": "Timeout",
                                "final_position": pos
                            }
                    break
                
                time.sleep(update_interval)
                
        except KeyboardInterrupt:
            print(f"\n⏹️  Smart grip stopped by user")
            for motor_id in initial_positions.keys():
                pos = motor_states[motor_id]["position"]
                for _ in range(5):
                    self.write_velocity(motor_id, 0)
                    self.write_position(motor_id, pos)
                    time.sleep(0.01)
                results[motor_id] = {
                    "success": False,
                    "reason": "User interrupted",
                    "final_position": pos
                }
        
        print("-" * 130)
        
        return {
            "success": len(results) > 0,
            "results": results,
            "active_motors": list(initial_positions.keys())
        }
    
    def emergency_stop_all(self) -> bool:
        """
        Emergency stop - set all motors to velocity 0 and then current positions
        """
        print("🚨 EMERGENCY STOP - Stopping all motors immediately")
        success = True
        
        # First set velocity to 0 for all motors
        for motor_id in self.motor_ids:
            if self.write_velocity(motor_id, 0):
                print(f"🛑 Motor {motor_id} velocity set to 0")
            else:
                print(f"❌ Failed to stop velocity of motor {motor_id}")
                success = False
        
        # Then set position to current position
        time.sleep(0.05)
        for motor_id in self.motor_ids:
            current_pos = self.read_position(motor_id)
            if current_pos is not None:
                if self.write_position(motor_id, current_pos):
                    print(f"🛑 Motor {motor_id} position locked at {current_pos}")
                else:
                    print(f"❌ Failed to lock position of motor {motor_id}")
                    success = False
            else:
                print(f"❌ Could not read position of motor {motor_id}")
                success = False
        
        return success
    
    def open_all_motors(self, show_status: bool = True, monitor_movement: bool = True) -> bool:
        """
        Open all motors to the fully open position
        
        Args:
            show_status: Show status messages
            monitor_movement: Monitor movement with real-time feedback
        
        Returns:
            True if all motors opened successfully
        """
        if show_status:
            print(f"\n🔓 Opening all motors to position {self.FULLY_OPEN}...")
        
        success = True
        for motor_id in self.motor_ids:
            if show_status:
                print(f"  Opening motor {motor_id}...")
            if not self.write_position(motor_id, self.FULLY_OPEN):
                print(f"  ❌ Failed to open motor {motor_id}")
                success = False
            else:
                if show_status:
                    print(f"  ✅ Motor {motor_id} open command sent")
        
        if success and monitor_movement:
            # Monitor all motors simultaneously
            target_positions = {motor_id: self.FULLY_OPEN for motor_id in self.motor_ids}
            self.monitor_all_motors_movement(target_positions)
        elif success:
            print("⏳ Waiting for movement to complete...")
            time.sleep(3.0)
        
        if show_status:
            if success:
                print("🔓 All motors opened successfully!")
            else:
                print("⚠️  Some motors failed to open")
        
        return success
    
    def close_all_motors(self, show_status: bool = True, monitor_movement: bool = True) -> bool:
        """
        Close all motors to the fully closed position
        
        Args:
            show_status: Show status messages
            monitor_movement: Monitor movement with real-time feedback
        
        Returns:
            True if all motors closed successfully
        """
        if show_status:
            print(f"\n🔒 Closing all motors to position {self.FULLY_CLOSED}...")
        
        success = True
        for motor_id in self.motor_ids:
            if show_status:
                print(f"  Closing motor {motor_id}...")
            if not self.write_position(motor_id, self.FULLY_CLOSED):
                print(f"  ❌ Failed to close motor {motor_id}")
                success = False
            else:
                if show_status:
                    print(f"  ✅ Motor {motor_id} close command sent")
        
        if success and monitor_movement:
            # Monitor all motors simultaneously
            target_positions = {motor_id: self.FULLY_CLOSED for motor_id in self.motor_ids}
            self.monitor_all_motors_movement(target_positions)
        elif success:
            print("⏳ Waiting for movement to complete...")
            time.sleep(3.0)
        
        if show_status:
            if success:
                print("🔒 All motors closed successfully!")
            else:
                print("⚠️  Some motors failed to close")
        
        return success
    
    def read_all_positions(self, show_status: bool = True) -> dict:
        """
        Read current positions of all motors
        
        Args:
            show_status: Show status messages
        
        Returns:
            Dictionary with motor_id: position pairs
        """
        positions = {}
        
        if show_status:
            print(f"\n📍 Reading positions of all motors...")
        
        for motor_id in self.motor_ids:
            position = self.read_position(motor_id)
            positions[motor_id] = position
            
            if show_status:
                if position is not None:
                    print(f"  Motor {motor_id}: {position}")
                else:
                    print(f"  Motor {motor_id}: ❌ Failed to read")
        
        return positions
    
    def move_single_motor(self, motor_id: int, position: int, monitor: bool = True) -> bool:
        """
        Move a single motor to specified position
        
        Args:
            motor_id: Motor ID
            position: Target position
            monitor: Monitor movement with real-time feedback
        """
        print(f"Moving motor {motor_id} to position {position}...")
        success = self.write_position(motor_id, position)
        
        if success and monitor:
            self.monitor_movement(motor_id, position)
        elif success:
            time.sleep(2.0)
        
        return success
    
    def monitor_movement(self, motor_id: int, target_position: int, update_interval: float = 0.1, max_duration: float = 10.0) -> bool:
        """
        Monitor motor movement with real-time feedback display
        
        Args:
            motor_id: Motor ID to monitor
            target_position: Target position for reference
            update_interval: Time between updates (seconds) - default 0.1s for 10Hz
            max_duration: Maximum monitoring time (seconds)
        
        Returns:
            True if target reached or monitoring completed
        """
        print(f"\n🔄 Monitoring Motor {motor_id} Movement")
        print(f"Target: {target_position}")
        print("-" * 80)
        print(f"{'Time':>6} | {'Position':>10} | {'Velocity':>10} | {'Torque':>10} | {'Current':>10} | {'Error':>10}")
        print("-" * 80)
        
        start_time = time.time()
        last_position = None
        stable_count = 0
        
        try:
            while True:
                current_time = time.time() - start_time
                
                # Read all feedback
                feedback = self.read_all_feedback(motor_id, show_output=False)
                
                if feedback['position'] is not None:
                    # Calculate position error
                    position_error = abs(target_position - feedback['position'])
                    
                    # Display current status
                    print(f"{current_time:6.1f} | {feedback['position']:>10} | {feedback['velocity']:>10} | {feedback['torque']:>10} | {feedback['current']:>10} | {position_error:>10}")
                    
                    # Check if motor has stopped (stable position)
                    if last_position is not None:
                        if abs(feedback['position'] - last_position) < 100:  # Adjust threshold as needed
                            stable_count += 1
                        else:
                            stable_count = 0
                    
                    # Stop monitoring if position is stable for multiple readings
                    if stable_count >= 5:  # 0.5 seconds of stability at 0.1s intervals
                        print(f"✅ Motor {motor_id} movement completed!")
                        break
                    
                    # Stop monitoring if target is reached (within tolerance)
                    if position_error < 1000:  # Adjust tolerance as needed
                        print(f"🎯 Motor {motor_id} reached target position!")
                        break
                    
                    last_position = feedback['position']
                else:
                    print(f"{current_time:6.1f} | {'ERROR':>10} | {'ERROR':>10} | {'ERROR':>10} | {'ERROR':>10} | {'ERROR':>10}")
                
                # Check timeout
                if current_time >= max_duration:
                    print(f"⏰ Monitoring timeout reached ({max_duration}s)")
                    break
                
                time.sleep(update_interval)
                
        except KeyboardInterrupt:
            print(f"\n⏹️  Monitoring stopped by user")
            return False
        
        print("-" * 80)
        return True
    
    def monitor_all_motors_movement(self, target_positions: dict, update_interval: float = 0.2, max_duration: float = 10.0) -> bool:
        """
        Monitor movement of all motors simultaneously
        
        Args:
            target_positions: Dict of {motor_id: target_position}
            update_interval: Time between updates (seconds) - default 0.2s for 5Hz
            max_duration: Maximum monitoring time (seconds)
        
        Returns:
            True if monitoring completed
        """
        print(f"\n🔄 Monitoring All Motors Movement")
        print(f"Targets: {target_positions}")
        print("-" * 120)
        
        header = f"{'Time':>6} |"
        for motor_id in sorted(target_positions.keys()):
            header += f" Motor {motor_id} (Pos/Vel/Torq/Curr) |"
        print(header)
        print("-" * 120)
        
        start_time = time.time()
        last_positions = {}
        stable_counts = {}
        
        # Initialize tracking variables
        for motor_id in target_positions.keys():
            last_positions[motor_id] = None
            stable_counts[motor_id] = 0
        
        try:
            while True:
                current_time = time.time() - start_time
                row = f"{current_time:6.1f} |"
                
                all_stable = True
                
                for motor_id in sorted(target_positions.keys()):
                    feedback = self.read_all_feedback(motor_id, show_output=False)
                    
                    if feedback is not None and feedback['position'] is not None:
                        target = target_positions[motor_id]
                        error = abs(target - feedback['position'])
                        
                        # Format: Position/Velocity/Torque/Current
                        motor_status = f" {feedback['position']:>7}/{feedback['velocity']:>6}/{feedback['torque']:>6}/{feedback['current']:>6} |"
                        row += motor_status
                        
                        # Check stability for this motor
                        if last_positions[motor_id] is not None:
                            if abs(feedback['position'] - last_positions[motor_id]) < 100:
                                stable_counts[motor_id] += 1
                            else:
                                stable_counts[motor_id] = 0
                        
                        last_positions[motor_id] = feedback['position']
                        
                        # Check if this motor reached target (either by position or stability)
                        if error > 1000 and stable_counts[motor_id] < 3:  # Still moving if error > 1000 AND not stable
                            all_stable = False
                    else:
                        row += f" {'ERROR':>7}/{'ERROR':>6}/{'ERROR':>6}/{'ERROR':>6} |"
                        all_stable = False
                
                print(row)
                
                # Stop if all motors reached targets
                if all_stable:
                    print("✅ All motors reached their targets!")
                    break
                
                # Check timeout
                if current_time >= max_duration:
                    print(f"⏰ Monitoring timeout reached ({max_duration}s)")
                    break
                
                time.sleep(update_interval)
                
        except KeyboardInterrupt:
            print(f"\n⏹️  Monitoring stopped by user")
            return False
        
        print("-" * 120)
        return True


def main():
    """Multi-motor control program with smart grip features - UPDATED VERSION"""
    # Configuration - CHANGE THESE TO MATCH YOUR SETUP
    COM_PORT = "COM10"          # Your COM port
    MOTOR_IDS = [1, 2, 3]       # Motor IDs to control
    FULLY_OPEN = 7000000        # Fully open position
    FULLY_CLOSED = -2000000          # Fully closed position
    
    # Create controller
    controller = JVLMotorController(COM_PORT, baudrate=19200, timeout=2.0)
    
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
            print("\n" + "="*50)
            print("🎮 JVL MOTOR CONTROLLER - UPDATED SMART GRIP")
            print("="*50)
            print("1. 🔓 Open all motors")
            print("2. 🔒 Close all motors") 
            print("3. 📍 Read all positions")
            print("4. 📊 Read all feedback (position, velocity, torque, current)")
            print("5. 🤏 Smart grip single motor (torque detection)")
            print("6. 🤏 Smart grip all motors (torque detection)") 
            print("7. 🎯 Move single motor")
            print("8. 🔍 Monitor single motor")
            print("9. ⚙️  Change position limits")
            print("10. 🔧 Change motor IDs")
            print("11. 🛡️  Change grip detection settings")
            print("12. 🔄 Toggle incremental/direct movement mode")
            print("99. 🚨 EMERGENCY STOP")
            print("0. ❌ Exit")
            print("="*50)
            
            choice = input("Enter your choice: ").strip()
            
            if choice == "1":
                # Open all motors
                controller.open_all_motors()
                
            elif choice == "2":
                # Close all motors
                controller.close_all_motors()
                
            elif choice == "3":
                # Read all positions
                controller.read_all_positions()
                
            elif choice == "4":
                # Read all feedback
                print("\n📊 Reading all feedback for all motors...")
                for motor_id in controller.motor_ids:
                    controller.read_all_feedback(motor_id)
                    
            elif choice == "5":
                # Smart grip single motor
                try:
                    motor_id = int(input(f"Enter motor ID {controller.motor_ids}: "))
                    if motor_id not in controller.motor_ids:
                        print(f"Invalid motor ID. Available: {controller.motor_ids}")
                        continue
                    
                    result = controller.smart_grip_single_motor(motor_id)
                    
                    if result["success"]:
                        print(f"✅ Smart grip successful!")
                        print(f"   Final position: {result['final_position']}")
                        print(f"   Reason: {result['reason']}")
                    else:
                        print(f"❌ Smart grip failed: {result['reason']}")
                        
                except ValueError:
                    print("Invalid input. Please enter numbers only.")
                    
            elif choice == "6":
                # Smart grip all motors
                print("🤏 Starting smart grip for all motors...")
                results = controller.smart_grip_all_motors()
                
                if results["success"]:
                    print("✅ Smart grip completed!")
                    for motor_id, result in results["results"].items():
                        status = "✅" if result["success"] else "❌"
                        print(f"   Motor {motor_id}: {status} {result['reason']} (pos: {result['final_position']})")
                else:
                    print("❌ Smart grip failed")
                
            elif choice == "7":
                # Move single motor
                try:
                    motor_id = int(input(f"Enter motor ID {controller.motor_ids}: "))
                    if motor_id not in controller.motor_ids:
                        print(f"Invalid motor ID. Available: {controller.motor_ids}")
                        continue
                    position = int(input("Enter target position: "))
                    
                    controller.move_single_motor(motor_id, position, monitor=True)
                        
                except ValueError:
                    print("Invalid input. Please enter numbers only.")
                    
            elif choice == "8":
                # Monitor single motor (without moving)
                try:
                    motor_id = int(input(f"Enter motor ID to monitor {controller.motor_ids}: "))
                    if motor_id not in controller.motor_ids:
                        print(f"Invalid motor ID. Available: {controller.motor_ids}")
                        continue
                    
                    # Get current position as target (just monitoring)
                    current_pos = controller.read_position(motor_id)
                    if current_pos is not None:
                        print(f"Monitoring motor {motor_id} (current position: {current_pos})")
                        print("Press Ctrl+C to stop monitoring...")
                        controller.monitor_movement(motor_id, current_pos, max_duration=30.0)
                    else:
                        print("Failed to read current position")
                        
                except ValueError:
                    print("Invalid input. Please enter numbers only.")
                    
            elif choice == "9":
                # Change position limits
                try:
                    print(f"Current limits: Open={controller.FULLY_OPEN}, Closed={controller.FULLY_CLOSED}")
                    new_open = int(input("Enter new FULLY OPEN position: "))
                    new_closed = int(input("Enter new FULLY CLOSED position: "))
                    controller.set_position_limits(new_open, new_closed)
                except ValueError:
                    print("Invalid input. Please enter numbers only.")
                    
            elif choice == "10":
                # Change motor IDs
                try:
                    print(f"Current motor IDs: {controller.motor_ids}")
                    ids_str = input("Enter motor IDs separated by commas (e.g., 1,2,3): ")
                    new_ids = [int(x.strip()) for x in ids_str.split(",")]
                    controller.set_motor_ids(new_ids)
                except ValueError:
                    print("Invalid input. Please enter numbers separated by commas.")
                    
            elif choice == "11":
                # Change detection settings
                try:
                    print(f"\nCurrent settings:")
                    print(f"  Detection mode: {controller.GRIP_DETECTION_MODE}")
                    print(f"  Velocity threshold: {controller.GRIP_VELOCITY_THRESHOLD}")
                    print(f"  Stall count: {controller.GRIP_STALL_COUNT}")
                    print(f"  Torque thresholds: {controller.MOTOR_SPECIFIC_THRESHOLDS}")
                    print("\nOptions:")
                    print("1. Switch detection mode (VELOCITY/TORQUE)")
                    print("2. Change velocity threshold")
                    print("3. Change stall count")
                    print("4. Change torque thresholds")
                    print("5. Disable grip detection")
                    print("6. Enable grip detection")
                    
                    sub_choice = input("Enter choice (1-6): ").strip()
                    
                    if sub_choice == "1":
                        current = controller.GRIP_DETECTION_MODE
                        new_mode = "TORQUE" if current == "VELOCITY" else "VELOCITY"
                        controller.GRIP_DETECTION_MODE = new_mode
                        print(f"Detection mode changed to: {new_mode}")
                    elif sub_choice == "2":
                        new_threshold = int(input("Enter new velocity threshold (current: {}): ".format(controller.GRIP_VELOCITY_THRESHOLD)))
                        controller.GRIP_VELOCITY_THRESHOLD = new_threshold
                        print(f"Velocity threshold set to {new_threshold}")
                    elif sub_choice == "3":
                        new_count = int(input("Enter new stall count (current: {}): ".format(controller.GRIP_STALL_COUNT)))
                        controller.GRIP_STALL_COUNT = new_count
                        print(f"Stall count set to {new_count}")
                    elif sub_choice == "4":
                        print("1. Change global torque threshold")
                        print("2. Change motor-specific threshold")
                        torque_choice = input("Enter choice: ").strip()
                        if torque_choice == "1":
                            new_threshold = int(input("Enter new global torque threshold: "))
                            controller.set_grip_limits(new_threshold)
                        elif torque_choice == "2":
                            motor_id = int(input("Enter motor ID: "))
                            threshold = int(input(f"Enter torque threshold for motor {motor_id}: "))
                            controller.MOTOR_SPECIFIC_THRESHOLDS[motor_id] = threshold
                            print(f"Motor {motor_id} threshold set to {threshold}")
                    elif sub_choice == "5":
                        controller.GRIP_DETECTION_ENABLED = False
                        print("Grip detection DISABLED")
                    elif sub_choice == "6":
                        controller.GRIP_DETECTION_ENABLED = True
                        print("Grip detection ENABLED")
                    
                except ValueError:
                    print("Invalid input. Please enter numbers only.")
                    
            elif choice == "12":
                # Toggle movement mode
                controller.USE_INCREMENTAL_MOVEMENT = not controller.USE_INCREMENTAL_MOVEMENT
                mode = "INCREMENTAL" if controller.USE_INCREMENTAL_MOVEMENT else "DIRECT"
                print(f"Movement mode changed to: {mode}")
                if controller.USE_INCREMENTAL_MOVEMENT:
                    print(f"Increment size: {controller.INCREMENT_SIZE}")
                    print(f"Increment delay: {controller.INCREMENT_DELAY}s")
                    
            elif choice == "99":
                # Emergency stop
                controller.emergency_stop_all()
                
            elif choice == "0":
                print("👋 Goodbye!")
                break
                
            else:
                print("❌ Invalid choice. Please try again.")
                
    except KeyboardInterrupt:
        print("\n🛑 Stopped by user")
    finally:
        controller.disconnect()


if __name__ == "__main__":
    main()