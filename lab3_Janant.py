from controller import Robot
import math

def run_robot(robot):
    time_step = 32
    max_speed = 6.28
    wheel_radius = 0.0205
    axle_length = 0.053

    # --- Motor ---
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    # --- Wheel sensors (encoder) ---
    left_sensor = robot.getDevice('left wheel sensor')
    right_sensor = robot.getDevice('right wheel sensor')
    left_sensor.enable(time_step)
    right_sensor.enable(time_step)

    # --- Proximity sensors ---
    ps_front_right = robot.getDevice('ps0')
    ps_front_left = robot.getDevice('ps7')
    ps_right = robot.getDevice('ps1')
    ps_left = robot.getDevice('ps6')
    ps_front_right.enable(time_step)
    ps_front_left.enable(time_step)
    ps_left.enable(time_step)
    ps_right.enable(time_step)

    x, z, theta = 0.0, 0.0, 0.0
    prev_left_enc = 0.0
    prev_right_enc = 0.0

    print("เริ่มเดินเกาะกำแพงขวาไปเรื่อย ๆ...")

    while robot.step(time_step) != -1:
        print(f"Pos: ({x:.3f}, {z:.3f}), Theta: {math.degrees(theta):.1f} deg")

        # Read encoders
        left_enc = left_sensor.getValue()
        right_enc = right_sensor.getValue()

        d_left = left_enc - prev_left_enc
        d_right = right_enc - prev_right_enc

        prev_left_enc = left_enc
        prev_right_enc = right_enc

        dl = d_left * wheel_radius
        dr = d_right * wheel_radius

        dc = (dl + dr) / 2.0
        dtheta = (dr - dl) / axle_length

        theta += dtheta
        x += dc * math.cos(theta)
        z += dc * math.sin(theta)

        # Read proximity sensor values
        front_val_right = ps_front_right.getValue()
        front_val_left = ps_front_left.getValue()
        right_val = ps_right.getValue()
        left_val = ps_left.getValue()
        
        print(f"front_val_right = {front_val_right:.2f}")
        print(f"front_val_left = {front_val_left:.2f}")
        print(f"right_val = {right_val:.2f}")
        print(f"left_val = {left_val:.2f}")
        
        # Set thresholds for detecting obstacles
        front_block_threshold = 100  # Obstacle in front if value > threshold
        wall_threshold = 100        # Wall detected on right if value < threshold

        # เช็คว่าเซ็นเซอร์ด้านหน้าเจอสิ่งกีดขวางหรือไม่
        if front_val_right > front_block_threshold or front_val_left > front_block_threshold:
            left_speed = max_speed * 0.1
            right_speed = max_speed * 0.5
        # ตรวจสอบว่าเซ็นเซอร์ขวาค้นพบกำแพงใกล้เกินไป
        elif right_val < wall_threshold:
            left_speed = max_speed * 0.5  # เพิ่มความเร็วของล้อซ้ายเพื่อเลี้ยวซ้าย
            right_speed = max_speed * 0.1  # ลดความเร็วของล้อขวาเพื่อหลีกเลี่ยงการชนกำแพง
        # หากไม่เจอกำแพงหรือสิ่งกีดขวาง
        else:
            left_speed = max_speed
            right_speed = max_speed
        
        # ตั้งค่าความเร็วของมอเตอร์
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

if __name__ == "__main__":
    robot = Robot()
    run_robot(robot)
