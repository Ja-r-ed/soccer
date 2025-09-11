import sensor, image, time, math, pyb, struct
import ustruct
from pyb import UART, LED

# EKF Implementation for Robot Localization
class ExtendedKalmanFilter:
    def __init__(self):
        # State vector: [x, y, theta, vx, vy]
        self.state = [0.0, 0.0, 0.0, 0.0, 0.0]  # x, y, theta, vx, vy
        
        # State covariance matrix (5x5)
        self.P = [[100.0 if i == j else 0.0 for j in range(5)] for i in range(5)]
        
        # Process noise covariance
        self.Q = [[0.1 if i == j else 0.0 for j in range(5)] for i in range(5)]
        self.Q[2][2] = 0.05  # Lower noise for orientation
        
        # Time step
        self.dt = 0.05  # 20Hz update rate
        
        # Field parameters (RoboCup Junior field in cm)
        self.FIELD_WIDTH = 122
        self.FIELD_HEIGHT = 183
        
    def predict(self, gyro_z, accel_x, accel_y):
        """Prediction step using IMU data"""
        x, y, theta, vx, vy = self.state
        
        # State prediction model
        new_x = x + vx * self.dt * math.cos(theta) - vy * self.dt * math.sin(theta)
        new_y = y + vx * self.dt * math.sin(theta) + vy * self.dt * math.cos(theta)
        new_theta = theta + gyro_z * self.dt
        
        # Simple velocity model with acceleration
        new_vx = vx + accel_x * self.dt * 0.1  # Scaled acceleration
        new_vy = vy + accel_y * self.dt * 0.1
        
        # Normalize angle
        new_theta = (new_theta + math.pi) % (2 * math.pi) - math.pi
        
        self.state = [new_x, new_y, new_theta, new_vx, new_vy]
        
        # Jacobian of state transition
        F = [[1, 0, -vx*self.dt*math.sin(theta) - vy*self.dt*math.cos(theta), self.dt*math.cos(theta), -self.dt*math.sin(theta)],
             [0, 1, vx*self.dt*math.cos(theta) - vy*self.dt*math.sin(theta), self.dt*math.sin(theta), self.dt*math.cos(theta)],
             [0, 0, 1, 0, 0],
             [0, 0, 0, 1, 0],
             [0, 0, 0, 0, 1]]
        
        # Covariance prediction: P = F*P*F^T + Q
        self.P = self.matrix_add(self.matrix_multiply(self.matrix_multiply(F, self.P), self.matrix_transpose(F)), self.Q)
    
    def update_tof(self, distance, sensor_angle, boundary_type):
        """Update with Time-of-Flight sensor measurement"""
        x, y, theta = self.state[0], self.state[1], self.state[2]
        
        # Calculate expected measurement based on current state
        sensor_world_angle = theta + sensor_angle
        
        # Determine expected distance to boundary
        if boundary_type == 'front':  # Front boundary (y = FIELD_HEIGHT/2)
            expected_dist = (self.FIELD_HEIGHT/2 - y) / math.cos(sensor_world_angle)
        elif boundary_type == 'back':  # Back boundary (y = -FIELD_HEIGHT/2)
            expected_dist = (-self.FIELD_HEIGHT/2 - y) / math.cos(sensor_world_angle)
        elif boundary_type == 'left':  # Left boundary (x = -FIELD_WIDTH/2)
            expected_dist = (-self.FIELD_WIDTH/2 - x) / math.sin(sensor_world_angle)
        elif boundary_type == 'right':  # Right boundary (x = FIELD_WIDTH/2)
            expected_dist = (self.FIELD_WIDTH/2 - x) / math.sin(sensor_world_angle)
        else:
            return
        
        if expected_dist < 0 or expected_dist > 200:  # Sanity check
            return
            
        # Measurement innovation
        innovation = distance - expected_dist
        
        # Measurement Jacobian (simplified)
        H = [[-math.sin(sensor_world_angle), -math.cos(sensor_world_angle), 
              -distance * math.cos(sensor_world_angle - theta), 0, 0]]
        
        # Innovation covariance
        R = 2.0  # ToF sensor noise
        S = self.matrix_add([[R]], self.matrix_multiply(self.matrix_multiply(H, self.P), self.matrix_transpose(H)))
        
        # Kalman gain
        K = self.matrix_multiply(self.matrix_multiply(self.P, self.matrix_transpose(H)), self.matrix_inverse(S))
        
        # State update
        for i in range(5):
            self.state[i] += K[i][0] * innovation
            
        # Covariance update
        I = [[1 if i == j else 0 for j in range(5)] for i in range(5)]
        KH = self.matrix_multiply(K, H)
        self.P = self.matrix_multiply(self.matrix_subtract(I, KH), self.P)
    
    def update_vision(self, landmark_x, landmark_y, measured_x, measured_y):
        """Update with vision landmark measurement"""
        x, y, theta = self.state[0], self.state[1], self.state[2]
        
        # Expected measurement in robot frame
        dx = landmark_x - x
        dy = landmark_y - y
        
        # Rotate to robot frame
        expected_x = dx * math.cos(-theta) - dy * math.sin(-theta)
        expected_y = dx * math.sin(-theta) + dy * math.cos(-theta)
        
        # Innovation
        innovation_x = measured_x - expected_x
        innovation_y = measured_y - expected_y
        
        # Simplified Jacobian for vision update
        H = [[-math.cos(-theta), -math.sin(-theta), dx*math.sin(-theta) + dy*math.cos(-theta), 0, 0],
             [math.sin(-theta), -math.cos(-theta), dx*math.cos(-theta) - dy*math.sin(-theta), 0, 0]]
        
        # Vision measurement noise
        R = [[4.0, 0], [0, 4.0]]  # Camera pixel uncertainty
        
        # Standard EKF update equations
        S = self.matrix_add(R, self.matrix_multiply(self.matrix_multiply(H, self.P), self.matrix_transpose(H)))
        K = self.matrix_multiply(self.matrix_multiply(self.P, self.matrix_transpose(H)), self.matrix_inverse(S))
        
        # Update state
        innovation = [innovation_x, innovation_y]
        for i in range(5):
            for j in range(2):
                self.state[i] += K[i][j] * innovation[j]
        
        # Update covariance
        I = [[1 if i == j else 0 for j in range(5)] for i in range(5)]
        KH = self.matrix_multiply(K, H)
        self.P = self.matrix_multiply(self.matrix_subtract(I, KH), self.P)
    
    # Matrix operations (simplified implementations)
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
    
    def matrix_inverse(self, A):
        # Simple 2x2 matrix inverse for measurement updates
        if len(A) == 2:
            det = A[0][0] * A[1][1] - A[0][1] * A[1][0]
            if abs(det) < 1e-6:
                return A  # Return original if singular
            return [[A[1][1]/det, -A[0][1]/det], [-A[1][0]/det, A[0][0]/det]]
        # For 1x1 case
        elif len(A) == 1:
            return [[1.0/A[0][0]] if abs(A[0][0]) > 1e-6 else [[1.0]]]
        return A

# Main robot controller class
class RoboCupRobot:
    def __init__(self):
        # Initialize camera
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)  # 320x240
        sensor.skip_frames(time=2000)
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(False)
        
        # Initialize UARTs for ESP32 communication
        self.esp32_tof = UART(3, 115200)  # ESP32 with ToF sensors
        self.esp32_aux = UART(1, 115200)  # ESP32 auxiliary
        
        # Initialize IMU (I2C)
        self.i2c = pyb.I2C(2, pyb.I2C.MASTER)
        self.imu_addr = 0x6A  # LSM6DSOX address
        
        # Initialize EKF
        self.ekf = ExtendedKalmanFilter()
        
        # Goal detection parameters
        self.goal_thresholds = [(0, 100, -128, 127, -128, 127)]  # Blue goal LAB
        
        # Timing
        self.last_time = time.ticks_ms()
        
    def read_imu(self):
        """Read IMU data from Adafruit 9-DOF"""
        try:
            # Read gyroscope (assuming LSM6DSOX)
            gyro_data = self.i2c.mem_read(6, self.imu_addr, 0x22)  # OUTX_L_G register
            gx = ustruct.unpack('<h', gyro_data[0:2])[0] * 0.00875  # Convert to deg/s
            gy = ustruct.unpack('<h', gyro_data[2:4])[0] * 0.00875
            gz = ustruct.unpack('<h', gyro_data[4:6])[0] * 0.00875
            
            # Read accelerometer
            accel_data = self.i2c.mem_read(6, self.imu_addr, 0x28)  # OUTX_L_A register
            ax = ustruct.unpack('<h', accel_data[0:2])[0] * 0.061  # Convert to m/s²
            ay = ustruct.unpack('<h', accel_data[2:4])[0] * 0.061
            az = ustruct.unpack('<h', accel_data[4:6])[0] * 0.061
            
            # Convert gyro to rad/s
            gz_rad = gz * math.pi / 180.0
            
            return gz_rad, ax, ay
        except:
            return 0.0, 0.0, 0.0
    
    def read_tof_data(self):
        """Read ToF sensor data from ESP32"""
        if self.esp32_tof.any():
            try:
                data = self.esp32_tof.read()
                if len(data) >= 8:  # Expecting 2 distances (2 sensors * 4 bytes each)
                    dist1 = ustruct.unpack('<f', data[0:4])[0]
                    dist2 = ustruct.unpack('<f', data[4:8])[0]
                    return dist1, dist2
            except:
                pass
        return None, None
    
    def detect_goals(self, img):
        """Detect goal posts using color thresholding"""
        goals = []
        
        # Find blue regions (opponent goal)
        for blob in img.find_blobs(self.goal_thresholds, pixels_threshold=100, area_threshold=100):
            if blob.area() > 500:  # Minimum area for goal post
                # Convert pixel coordinates to field coordinates (simplified)
                pixel_x = blob.cx() - 160  # Center relative to image center
                pixel_y = 120 - blob.cy()  # Invert Y axis
                
                # Simple perspective transformation (needs calibration)
                field_x = pixel_x * 0.5  # Scale factor
                field_y = 50 + pixel_y * 0.3  # Distance estimation
                
                goals.append((field_x, field_y, 'blue'))
        
        return goals
    
    def run(self):
        """Main control loop"""
        print("Starting RoboCup localization system...")
        
        while True:
            current_time = time.ticks_ms()
            dt = time.ticks_diff(current_time, self.last_time) / 1000.0
            
            if dt >= 0.05:  # 20Hz update rate
                # Read IMU
                gyro_z, accel_x, accel_y = self.read_imu()
                
                # EKF Prediction step
                self.ekf.predict(gyro_z, accel_x, accel_y)
                
                # Read ToF sensors
                tof_dist1, tof_dist2 = self.read_tof_data()
                
                # Update with ToF measurements
                if tof_dist1 and tof_dist1 < 100:  # Valid measurement
                    self.ekf.update_tof(tof_dist1, -0.785, 'front')  # -45 degrees
                if tof_dist2 and tof_dist2 < 100:
                    self.ekf.update_tof(tof_dist2, 0.785, 'front')   # +45 degrees
                
                # Vision processing
                img = sensor.snapshot()
                goals = self.detect_goals(img)
                
                # Update with vision measurements
                for goal_x, goal_y, color in goals:
                    if color == 'blue':
                        # Known blue goal position
                        landmark_x, landmark_y = 0, 91.5  # Field coordinates
                        self.ekf.update_vision(landmark_x, landmark_y, goal_x, goal_y)
                
                # Output current position
                x, y, theta = self.ekf.state[0], self.ekf.state[1], self.ekf.state[2]
                print(f"Position: ({x:.1f}, {y:.1f}) cm, Angle: {theta*180/math.pi:.1f}°")
                
                # Send position to ESP32s if needed
                pos_data = ustruct.pack('<fff', x, y, theta)
                self.esp32_aux.write(pos_data)
                
                self.last_time = current_time
                
                # Visual feedback
                img.draw_cross(160, 120, color=(255, 0, 0))  # Robot center
                for goal_x, goal_y, color in goals:
                    img.draw_circle(int(goal_x + 160), int(120 - goal_y), 10, 
                                  color=(0, 255, 0) if color == 'blue' else (255, 255, 0))

# Initialize and run
if __name__ == "__main__":
    robot = RoboCupRobot()
    robot.run()