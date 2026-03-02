from __future__ import print_function
import time
from dual_g2_hpmd_rpi import motors, MAX_SPEED

#take the encoader value compare to the driver value, if it is different change the driver module value
class PIDController:
    def __init__(self, kp, ki, kd, output_min=-MAX_SPEED, output_max=MAX_SPEED):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        
        self.prev_error = 0
        self.integral = 0
        self.prev_time = None
    
    def compute(self, setpoint, measured_value):
        current_time = time.time()
        
        if self.prev_time is None:
            self.prev_time = current_time
            dt = 0.01 
        else:
            dt = current_time - self.prev_time
            if dt <= 0:
                dt = 0.01
        
        error = setpoint - measured_value
        

        p_term = self.kp * error
        

        self.integral += error * dt

        max_integral = self.output_max / (self.ki + 0.001) 
        self.integral = max(-max_integral, min(max_integral, self.integral))
        i_term = self.ki * self.integral

        d_term = self.kd * (error - self.prev_error) / dt
        
        output = p_term + i_term + d_term
        
        output = max(self.output_min, min(self.output_max, output))
        
        self.prev_error = error
        self.prev_time = current_time
        
        return output
    
    def reset(self):
        self.prev_error = 0
        self.integral = 0
        self.prev_time = None

def drive_forward_pid(target_speed, duration, encoder_feedback_func):
    pid_motor1 = PIDController(kp=2.0, ki=0.5, kd=0.1)
    pid_motor2 = PIDController(kp=2.0, ki=0.5, kd=0.1)
    
    start_time = time.time()
    
    while (time.time() - start_time) < duration:
        current_speed_m1, current_speed_m2 = encoder_feedback_func()
        
        output_m1 = pid_motor1.compute(target_speed, current_speed_m1)
        output_m2 = pid_motor2.compute(target_speed, current_speed_m2)

        motors.motor1.setSpeed(int(output_m1))
        motors.motor2.setSpeed(int(output_m2))

        print(f"M1: Target={target_speed}, Current={current_speed_m1:.1f}, Output={output_m1:.1f}")
        print(f"M2: Target={target_speed}, Current={current_speed_m2:.1f}, Output={output_m2:.1f}")
        
        time.sleep(0.05)
    
    motors.motor1.setSpeed(0)
    motors.motor2.setSpeed(0)

def drive_straight_pid(target_speed, duration, encoder_feedback_func):
    speed_pid_m1 = PIDController(kp=2.0, ki=0.5, kd=0.1)
    speed_pid_m2 = PIDController(kp=2.0, ki=0.5, kd=0.1)
    heading_pid = PIDController(kp=1.0, ki=0.0, kd=0.5, 
                                output_min=-100, output_max=100)
    
    start_time = time.time()
    
    while (time.time() - start_time) < duration:
        # Get current speeds
        speed_m1, speed_m2 = encoder_feedback_func()

        heading_error = speed_m1 - speed_m2
        heading_correction = heading_pid.compute(0, heading_error)
        
        output_m1 = speed_pid_m1.compute(target_speed, speed_m1) - heading_correction
        output_m2 = speed_pid_m2.compute(target_speed, speed_m2) + heading_correction
        
        output_m1 = max(-MAX_SPEED, min(MAX_SPEED, output_m1))
        output_m2 = max(-MAX_SPEED, min(MAX_SPEED, output_m2))
        
        motors.motor1.setSpeed(int(output_m1))
        motors.motor2.setSpeed(int(output_m2))
        
        print(f"Speeds: M1={speed_m1:.1f}, M2={speed_m2:.1f}, Correction={heading_correction:.1f}")
        
        time.sleep(0.05)
    
    motors.motor1.setSpeed(0)
    motors.motor2.setSpeed(0)


if __name__ == "__main__":
    try:
        motors.enable()
        # print("Smooth ramping...")
        # drive_forward_smooth(300, 5)
        
        time.sleep(1)
        
        def get_encoder_speeds():
            return (100, 105)  # Example values
        
        print("PID with encoder feedback...")
        drive_forward_pid(200, 5, get_encoder_speeds)
        
        print("Done!")
        
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        motors.motor1.setSpeed(0)
        motors.motor2.setSpeed(0)
        motors.forceStop()
