import pygame
import math
import sys

# --- PID Controller ---
class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.prev_error = 0
        self.integral = 0

    def update(self, setpoint, measured, dt):
        error = setpoint - measured
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

# --- Robot Drive ---
class XDriveRobot:
    def __init__(self, x, y, angle=0):
        self.x, self.y, self.angle = x, y, angle
        self.wheel_angles = [45, -45, -135, 135]  # FL, FR, BR, BL
        self.wheel_pos = [
            (-30, -30), (30, -30), (30, 30), (-30, 30)
        ]
        self.wheel_speeds = [0, 0, 0, 0]
        self.dribbler_on = False

    def drive(self, direction_deg, speed, rotation):
        # Convert direction to radians, adjust for robot heading
        direction_rad = math.radians(direction_deg)
        vx = speed * math.cos(direction_rad)
        vy = speed * math.sin(direction_rad)  # pygame y axis is down

        # X-drive kinematics
        # FL, FR, BR, BL
        self.wheel_speeds[0] = vx * math.sin(math.radians(45)) + vy * math.cos(math.radians(45)) + rotation
        self.wheel_speeds[1] = vx * math.sin(math.radians(-45)) + vy * math.cos(math.radians(-45)) + rotation
        self.wheel_speeds[2] = vx * math.sin(math.radians(-135)) + vy * math.cos(math.radians(-135)) + rotation
        self.wheel_speeds[3] = vx * math.sin(math.radians(135)) + vy * math.cos(math.radians(135)) + rotation

        # Normalize speeds
        max_speed = max(abs(s) for s in self.wheel_speeds)
        if max_speed > 1:
            self.wheel_speeds = [s / max_speed for s in self.wheel_speeds]

        # Update robot position and angle
        # self.x += vx
        # self.y += vy
        # self.angle += rotation * 5  # scale rotation for visualization

    def draw(self, screen):
        # Draw robot body
        center = (int(self.x), int(self.y))
        pygame.draw.circle(screen, (0, 100, 255), center, 40, 3)
        # Draw heading
        heading = (
            int(self.x + 40 * math.cos(math.radians(-self.angle + 90))),
            int(self.y - 40 * math.sin(math.radians(-self.angle + 90)))
        )
        pygame.draw.line(screen, (255, 0, 0), center, heading, 4)

        # Draw wheels and arrows
        for i, (wx, wy) in enumerate(self.wheel_pos):
            # Rotate wheel position
            angle_rad = math.radians(self.angle)
            rx = wx * math.cos(angle_rad) - wy * math.sin(angle_rad)
            ry = wx * math.sin(angle_rad) + wy * math.cos(angle_rad)
            wheel_center = (int(self.x + rx), int(self.y + ry))
            pygame.draw.circle(screen, (0, 255, 0), wheel_center, 10, 2)

            # Draw arrow for wheel direction
            speed = self.wheel_speeds[i]
            if abs(speed) > 0.05:
                arrow_len = 25 * abs(speed)
                wheel_angle = self.wheel_angles[i] - self.angle
                if speed < 0:
                    wheel_angle += 180
                arrow_rad = math.radians(wheel_angle)
                arrow_end = (
                    int(wheel_center[0] + arrow_len * math.cos(arrow_rad)),
                    int(wheel_center[1] - arrow_len * math.sin(arrow_rad))
                )
                pygame.draw.line(screen, (255, 255, 0), wheel_center, arrow_end, 4)
                # Arrowhead
                pygame.draw.circle(screen, (255, 255, 0), arrow_end, 4)

        # Draw dribbler
        if self.dribbler_on:
            drib_x = int(self.x + 50 * math.cos(math.radians(-self.angle + 90)))
            drib_y = int(self.y - 50 * math.sin(math.radians(-self.angle + 90)))
            pygame.draw.circle(screen, (255, 100, 0), (drib_x, drib_y), 10)

# --- Main Simulation ---
def main():
    pygame.init()
    screen = pygame.display.set_mode((800, 600))
    pygame.display.set_caption("RoboCup X-Drive Simulator")
    clock = pygame.time.Clock()

    robot = XDriveRobot(400, 300)
    pid = PID(0.1, 0.0, 0.01)

    translation_dir = 0
    speed = 0
    rotation = 0

    running = True
    while running:
        dt = clock.tick(60) / 1000.0

        # Input handling
        keys = pygame.key.get_pressed()
        translation_dir = None
        speed = 0
        rotation = 0

        # WASD for translation
        if keys[pygame.K_w]:
            translation_dir = 0
            speed = 5
        if keys[pygame.K_s]:
            translation_dir = 180
            speed = 5
        if keys[pygame.K_a]:
            translation_dir = 270
            speed = 5
        if keys[pygame.K_d]:
            translation_dir = 90
            speed = 5
        # Diagonals
        if keys[pygame.K_w] and keys[pygame.K_a]:
            translation_dir = 315
        if keys[pygame.K_w] and keys[pygame.K_d]:
            translation_dir = 45
        if keys[pygame.K_s] and keys[pygame.K_a]:
            translation_dir = 225
        if keys[pygame.K_s] and keys[pygame.K_d]:
            translation_dir = 135

        # Q/E for rotation
        if keys[pygame.K_q]:
            rotation = -0.5
        if keys[pygame.K_e]:
            rotation = 0.5

        # Spacebar for dribbler
        robot.dribbler_on = keys[pygame.K_SPACE]

        # Drive robot
        if translation_dir is not None:
            robot.drive(translation_dir, speed, rotation)
        else:
            robot.drive(0, 0, rotation)

        # Draw everything
        screen.fill((30, 30, 30))
        robot.draw(screen)
        pygame.display.flip()

        # Quit
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()