import sensor, image, time, math, pyb, struct
import ustruct
from pyb import UART, LED

# EKF Implementation for X-Drive Omniwheel Robot Localization
class ExtendedKalmanFilter:
    def __init__(self):
        # State vector: [x, y, theta, vx, vy, omega]
        self.state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # x, y, theta, vx, vy, omega
        
        # State covariance matrix (6x6)
        self.P = [[100.0 if i == j else 0.0 for j in range(6)] for i in range(6)]
        
        # Process noise covariance
        self.Q = [[0.0 for j in range(6)] for i in range(6)]
        self.Q[0][0] = 0.1   # Position x
        self.Q[1][1] = 0.1   # Position y
        self.Q[2][2] = 0.05  # Orientation (lower noise for gyro)
        self.Q[3][3] = 0.2   # Velocity x
        self.Q[4][4] = 0.2   # Velocity y
        self.Q[5][5] = 0.1   # Angular velocity
        
        # Time step
        self.dt = 0.05  # 20Hz update rate
        
        # Field parameters (RoboCup Junior field in cm)
        self.FIELD_WIDTH = 122
        self.FIELD_HEIGHT = 183
        
        # Ball tracking
        self.ball_position = [0.0, 0.0]
        self.ball_visible = False
        
    def predict(self, gyro_z, accel_x, accel_y):
        """Prediction step using IMU data"""
        x, y, theta, vx, vy, omega = self.state
        
        # X-drive kinematics - robot can move in any direction
        # State prediction model for holonomic drive
        new_x = x + (vx * math.cos(theta) - vy * math.sin(theta)) * self.dt
        new_y = y + (vx * math.sin(theta) + vy * math.cos(theta)) * self.dt
        new_theta = theta + omega * self.dt
        
        # Velocity model with acceleration input
        new_vx = vx * 0.9 + accel_x * self.dt * 0.1  # Add damping
        new_vy = vy * 0.9 + accel_y * self.dt * 0.1
        new_omega = omega * 0.9 + gyro_z * 0.1  # Smooth angular velocity
        
        # Normalize angle
        new_theta = (new_theta + math.pi) % (2 * math.pi) - math.pi
        
        self.state = [new_x, new_y, new_theta, new_vx, new_vy, new_omega]
        
        # Jacobian of state transition (6x6)
        F = [[1, 0, -vx*self.dt*math.sin(theta) - vy*self.dt*math.cos(theta), self.dt*math.cos(theta), -self.dt*math.sin(theta), 0],
             [0, 1, vx*self.dt*math.cos(theta) - vy*self.dt*math.sin(theta), self.dt*math.sin(theta), self.dt*math.cos(theta), 0],
             [0, 0, 1, 0, 0, self.dt],
             [0, 0, 0, 0.9, 0, 0],
             [0, 0, 0, 0, 0.9, 0],
             [0, 0, 0, 0, 0, 0.9]]
        
        # Covariance prediction: P = F*P*F^T + Q
        self.P = self.matrix_add(self.matrix_multiply(self.matrix_multiply(F, self.P), self.matrix_transpose(F)), self.Q)
    
    def update_tof(self, distances, sensor_positions):
        """Update with multiple ToF sensor measurements
        distances: list of distance measurements
        sensor_positions: list of (angle_offset, boundary_type) tuples
        """
        x, y, theta = self.state[0], self.state[1], self.state[2]
        
        for i, (distance, (angle_offset, boundary_type)) in enumerate(zip(distances, sensor_positions)):
            if distance < 0 or distance > 150:  # Invalid reading
                continue
                
            # Calculate sensor world angle
            sensor_world_angle = theta + angle_offset
            
            # Calculate expected distance to boundary
            expected_dist = self.calculate_expected_tof_distance(x, y, sensor_world_angle, boundary_type)
            
            if expected_dist < 0 or expected_dist > 200:
                continue
                
            # Measurement innovation
            innovation = distance - expected_dist
            
            # Measurement Jacobian (simplified for single ToF)
            cos_angle = math.cos(sensor_world_angle)
            sin_angle = math.sin(sensor_world_angle)
            
            H = [[-sin_angle, -cos_angle, -distance * math.cos(sensor_world_angle - theta), 0, 0, 0]]
            
            # Innovation covariance
            R = 4.0  # ToF sensor noise
            S = R + self.matrix_multiply(self.matrix_multiply(H, self.P), self.matrix_transpose(H))[0][0]
            
            if abs(S) < 1e-6:
                continue
                
            # Kalman gain
            K = [[self.P[j][0] * H[0][0] / S] for j in range(6)]
            
            # State update
            for j in range(6):
                self.state[j] += K[j][0] * innovation
                
            # Covariance update (Joseph form for numerical stability)
            I = [[1 if i == j else 0 for j in range(6)] for i in range(6)]
            KH = [[K[i][0] * H[0][j] for j in range(6)] for i in range(6)]
            IKH = self.matrix_subtract(I, KH)
            self.P = self.matrix_multiply(self.matrix_multiply(IKH, self.P), self.matrix_transpose(IKH))
            # Add measurement noise contribution
            KRKt = [[K[i][0] * R * K[j][0] for j in range(6)] for i in range(6)]
            self.P = self.matrix_add(self.P, KRKt)
    
    def calculate_expected_tof_distance(self, x, y, angle, boundary_type):
        """Calculate expected ToF distance to field boundary"""
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        
        if boundary_type == 'front':  # Front boundary (y = FIELD_HEIGHT/2)
            if cos_a != 0:
                return (self.FIELD_HEIGHT/2 - y) / cos_a
        elif boundary_type == 'back':  # Back boundary (y = -FIELD_HEIGHT/2)
            if cos_a != 0:
                return (-self.FIELD_HEIGHT/2 - y) / cos_a
        elif boundary_type == 'left':  # Left boundary (x = -FIELD_WIDTH/2)
            if sin_a != 0:
                return (-self.FIELD_WIDTH/2 - x) / sin_a
        elif boundary_type == 'right':  # Right boundary (x = FIELD_WIDTH/2)
            if sin_a != 0:
                return (self.FIELD_WIDTH/2 - x) / sin_a
                
        return -1  # Invalid
    
    def update_vision_landmark(self, landmark_world_pos, measured_relative_pos):
        """Update with vision landmark measurement"""
        x, y, theta = self.state[0], self.state[1], self.state[2]
        
        # Expected measurement in robot frame
        dx = landmark_world_pos[0] - x
        dy = landmark_world_pos[1] - y
        
        # Rotate to robot frame
        cos_t = math.cos(-theta)
        sin_t = math.sin(-theta)
        expected_x = dx * cos_t - dy * sin_t
        expected_y = dx * sin_t + dy * cos_t
        
        # Innovation
        innovation = [measured_relative_pos[0] - expected_x,
                     measured_relative_pos[1] - expected_y]
        
        # Measurement Jacobian
        H = [[-cos_t, -sin_t, dx*sin_t + dy*cos_t, 0, 0, 0],
             [sin_t, -cos_t, dx*cos_t - dy*sin_t, 0, 0, 0]]
        
        # Vision measurement noise
        R = [[16.0, 0], [0, 16.0]]  # Camera uncertainty
        
        # Standard EKF update
        S = self.matrix_add(R, self.matrix_multiply(self.matrix_multiply(H, self.P), self.matrix_transpose(H)))
        S_inv = self.matrix_inverse_2x2(S)
        
        if S_inv is None:
            return
            
        K = self.matrix_multiply(self.matrix_multiply(self.P, self.matrix_transpose(H)), S_inv)
        
        # Update state
        for i in range(6):
            for j in range(2):
                self.state[i] += K[i][j] * innovation[j]
        
        # Update covariance
        I = [[1 if i == j else 0 for j in range(6)] for i in range(6)]
        KH = self.matrix_multiply(K, H)
        self.P = self.matrix_multiply(self.matrix_subtract(I, KH), self.P)
    
    # Matrix operations
    def matrix_multiply(self, A, B):
        rows_A, cols_A = len(A), len(A[0])
        rows_B, cols_B = len(B), len(B[0])
        result = [[0 for _ in range(cols_B)] for _ in range(rows_A)]
        
        for i in range(rows_A):
            for j in range(cols_B):
                for k in range(cols_A):
                    result[i][j] += A[i][k] * B[k][j]
        return result
    
    def matrix_transpose(self, A):
        return [[A[j][i] for j in range(len(A))] for i in range(len(A[0]))]
    
    def matrix_add(self, A, B):
        return [[A[i][j] + B[i][j] for j in range(len(A[0]))] for i in range(len(A))]
    
    def matrix_subtract(self, A, B):
        return [[A[i][j] - B[i][j] for j in range(len(A[0]))] for i in range(len(A))]
    
    def matrix_inverse_2x2(self, A):
        det = A[0][0] * A[1][1] - A[0][1] * A[1][0]
        if abs(det) < 1e-6:
            return None
        return [[A[1][1]/det, -A[0][1]/det], [-A[1][0]/det, A[0][0]/det]]

# Main robot controller class for X-drive robot
class RoboCupXDriveRobot:
    def __init__(self):
        # Initialize camera
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)  # 320x240
        sensor.skip_frames(time=2000)
        sensor.set_auto_gain(False, gain_db=10)
        sensor.set_auto_whitebal(False)
        sensor.set_auto_exposure(False, exposure_us=10000)
        
        # Initialize UARTs for ESP32 communication
        self.esp32_sensors = UART(1, 115200)  # ESP32 with sensors
        self.esp32_motors = UART(3, 115200)   # ESP32 with motors
        
        # Initialize IMU (I2C)
        self.i2c = pyb.I2C(2, pyb.I2C.MASTER)
        self.imu_addr = 0x6A  # LSM6DSOX address (adjust if needed)
        
        # Initialize EKF
        self.ekf = ExtendedKalmanFilter()
        
        # Vision parameters
        self.goal_blue_threshold = [(0, 15, -25, 25, -40, -10)]    # Blue goal
        self.goal_yellow_threshold = [(30, 100, -10, 10, 20, 60)] # Yellow goal
        self.ball_threshold = [(20, 100, -20, 20, 20, 70)]        # Orange ball
        
        # ToF sensor configuration (4 sensors on X-drive robot)
        self.tof_positions = [
            (math.pi/4, 'front'),      # Front-right sensor
            (3*math.pi/4, 'front'),    # Front-left sensor  
            (-math.pi/4, 'back'),      # Back-right sensor
            (-3*math.pi/4, 'back')     # Back-left sensor
        ]
        
        # Timing
        self.last_time = time.ticks_ms()
        
        # Initialize IMU
        self.init_imu()
        
    def init_imu(self):
        """Initialize the LSM6DSOX IMU"""
        try:
            # Configure gyroscope (ODR = 208 Hz, FS = 2000 dps)
            self.i2c.mem_write(0x6C, self.imu_addr, 0x11)  # CTRL2_G
            # Configure accelerometer (ODR = 208 Hz, FS = 4g)
            self.i2c.mem_write(0x60, self.imu_addr, 0x10)  # CTRL1_XL
            print("IMU initialized successfully")
        except:
            print("IMU initialization failed")
    
    def read_imu(self):
        """Read IMU data from Adafruit 9-DOF"""
        try:
            # Read gyroscope
            gyro_data = self.i2c.mem_read(6, self.imu_addr, 0x22)
            gx = ustruct.unpack('<h', gyro_data[0:2])[0] * 0.07 * math.pi / 180.0  # Convert to rad/s
            gy = ustruct.unpack('<h', gyro_data[2:4])[0] * 0.07 * math.pi / 180.0
            gz = ustruct.unpack('<h', gyro_data[4:6])[0] * 0.07 * math.pi / 180.0
            
            # Read accelerometer
            accel_data = self.i2c.mem_read(6, self.imu_addr, 0x28)
            ax = ustruct.unpack('<h', accel_data[0:2])[0] * 0.122  # Convert to m/s²
            ay = ustruct.unpack('<h', accel_data[2:4])[0] * 0.122
            az = ustruct.unpack('<h', accel_data[4:6])[0] * 0.122
            
            return gz, ax, ay  # Return z-gyro and x,y accelerations
        except:
            return 0.0, 0.0, 0.0
    
    def read_sensor_data(self):
        """Read sensor data from sensors ESP32"""
        if self.esp32_sensors.any():
            try:
                data = self.esp32_sensors.read()
                if len(data) >= 16:  # 4 ToF distances (4 bytes each)
                    distances = []
                    for i in range(4):
                        dist = ustruct.unpack('<f', data[i*4:(i+1)*4])[0]
                        distances.append(dist if dist > 0 else -1)
                    return distances
            except:
                pass
        return [-1, -1, -1, -1]
    
    def detect_objects(self, img):
        """Detect goals, ball, and field lines"""
        objects = {
            'blue_goal': None,
            'yellow_goal': None,
            'ball': None,
            'field_lines': []
        }
        
        # Detect blue goal
        blue_blobs = img.find_blobs(self.goal_blue_threshold, pixels_threshold=200, area_threshold=200)
        if blue_blobs:
            largest_blue = max(blue_blobs, key=lambda b: b.area())
            if largest_blue.area() > 500:
                objects['blue_goal'] = self.pixel_to_field_coords(largest_blue.cx(), largest_blue.cy(), largest_blue.area())
                img.draw_rectangle(largest_blue.rect(), color=(0, 0, 255))
        
        # Detect yellow goal
        yellow_blobs = img.find_blobs(self.goal_yellow_threshold, pixels_threshold=200, area_threshold=200)
        if yellow_blobs:
            largest_yellow = max(yellow_blobs, key=lambda b: b.area())
            if largest_yellow.area() > 500:
                objects['yellow_goal'] = self.pixel_to_field_coords(largest_yellow.cx(), largest_yellow.cy(), largest_yellow.area())
                img.draw_rectangle(largest_yellow.rect(), color=(255, 255, 0))
        
        # Detect ball
        ball_blobs = img.find_blobs(self.ball_threshold, pixels_threshold=100, area_threshold=100)
        if ball_blobs:
            largest_ball = max(ball_blobs, key=lambda b: b.area())
            if largest_ball.area() > 200:
                ball_coords = self.pixel_to_field_coords(largest_ball.cx(), largest_ball.cy(), largest_ball.area())
                objects['ball'] = ball_coords
                self.ekf.ball_position = ball_coords
                self.ekf.ball_visible = True
                img.draw_circle(largest_ball.cx(), largest_ball.cy(), 15, color=(255, 0, 0))
            else:
                self.ekf.ball_visible = False
        else:
            self.ekf.ball_visible = False
        
        return objects
    
    def pixel_to_field_coords(self, pixel_x, pixel_y, area):
        """Convert pixel coordinates to field coordinates relative to robot"""
        # Center coordinates relative to image center
        rel_x = pixel_x - 160  # Image width/2
        rel_y = 120 - pixel_y  # Image height/2 (inverted Y)
        
        # Simple distance estimation based on blob area
        # These values need calibration for your specific camera setup
        if area > 2000:
            distance = 30  # Very close
        elif area > 1000:
            distance = 50  # Medium distance
        elif area > 500:
            distance = 80  # Far distance
        else:
            distance = 120  # Very far
        
        # Convert to robot-relative coordinates
        angle = math.atan2(rel_x, distance)  # Horizontal angle
        field_x = distance * math.sin(angle)
        field_y = distance * math.cos(angle)
        
        return [field_x, field_y]
    
    def send_robot_state(self):
        """Send current state to motor controller ESP32"""
        x, y, theta, vx, vy, omega = self.ekf.state
        
        # Pack state data
        state_data = ustruct.pack('<ffffff', x, y, theta, vx, vy, omega)
        
        # Add ball information
        if self.ekf.ball_visible:
            ball_data = ustruct.pack('<ff?', self.ekf.ball_position[0], self.ekf.ball_position[1], True)
        else:
            ball_data = ustruct.pack('<ff?', 0.0, 0.0, False)
        
        # Send combined data
        self.esp32_motors.write(state_data + ball_data)
    
    def run(self):
        """Main control loop"""
        print("Starting X-Drive RoboCup localization system...")
        
        while True:
            current_time = time.ticks_ms()
            dt = time.ticks_diff(current_time, self.last_time) / 1000.0
            
            if dt >= 0.05:  # 20Hz update rate
                # Read IMU
                gyro_z, accel_x, accel_y = self.read_imu()
                
                # EKF Prediction step
                self.ekf.predict(gyro_z, accel_x, accel_y)
                
                # Read ToF sensors
                tof_distances = self.read_sensor_data()
                
                # Update with ToF measurements
                valid_distances = []
                valid_positions = []
                for i, dist in enumerate(tof_distances):
                    if 5 < dist < 100:  # Valid range
                        valid_distances.append(dist)
                        valid_positions.append(self.tof_positions[i])
                
                if valid_distances:
                    self.ekf.update_tof(valid_distances, valid_positions)
                
                # Vision processing
                img = sensor.snapshot()
                objects = self.detect_objects(img)
                
                # Update with vision measurements
                if objects['blue_goal']:
                    # Known blue goal position (opponent goal)
                    self.ekf.update_vision_landmark([0, 91.5], objects['blue_goal'])
                
                if objects['yellow_goal']:
                    # Known yellow goal position (own goal)
                    self.ekf.update_vision_landmark([0, -91.5], objects['yellow_goal'])
                
                # Send state to motor controller
                self.send_robot_state()
                
                # Debug output
                x, y, theta = self.ekf.state[0], self.ekf.state[1], self.ekf.state[2]
                print(f"Pos: ({x:.1f},{y:.1f}) Θ:{theta*180/math.pi:.1f}° Ball:{self.ekf.ball_visible}")
                
                # Visual feedback
                img.draw_cross(160, 120, color=(0, 255, 0), size=10)  # Robot center
                
                self.last_time = current_time
                
            time.sleep_ms(10)  # Small delay

# Initialize and run
if __name__ == "__main__":
    robot = RoboCupXDriveRobot()
    robot.run()