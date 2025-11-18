import numpy as np
from scipy import signal

def robotSpeed2motorSpeed(robot_speed: tuple[float, float], wheel_radius: float, wheel_distance: float) -> tuple[float, float]:
    left_wheel_speed = (robot_speed[0]/wheel_radius) - (wheel_distance/(2*wheel_radius)) * robot_speed[1]
    right_wheel_speed = (robot_speed[0]/wheel_radius) + (wheel_distance/(2*wheel_radius)) * robot_speed[1]
    
    return left_wheel_speed, right_wheel_speed

def motorSpeed2pwm(motor_speed: tuple[float, float]) -> tuple[float, float]:
    left_bridge = 0.4182 * motor_speed[0]
    right_bridge = 0.4182 * motor_speed[1]

    return np.clip(left_bridge, -1, 1), np.clip(right_bridge, -1, 1)

def pwm2voltage(pwm: tuple[float, float], vin: float) -> tuple[float, float]:
    left_bridge = pwm[0] * vin
    right_bridge = pwm[1] * vin

    if abs(left_bridge) < 1.5:
        left_bridge = 0
    if abs(right_bridge) < 1.5:
        right_bridge = 0 

    return left_bridge, right_bridge

def voltage2motorSpeed(voltage: tuple[float, float]):
    global left_motor_dc, right_motor_dc
    
    left_wheel_speed = left_motor_dc.step(voltage[0])
    right_wheel_speed = right_motor_dc.step(voltage[1])

    return left_wheel_speed, right_wheel_speed

def motorSpeed2robotSpeed(motor_speed: tuple[float, float], wheel_radius: float, wheel_distance: float) -> tuple[float, float]:
    robot_speed = (wheel_radius/2) * motor_speed[0] + (wheel_radius/2) * motor_speed[1]
    robot_angular_vel = -(wheel_radius/wheel_distance) * motor_speed[0] + (wheel_radius/wheel_distance) * motor_speed[1]

    if abs(robot_speed) <= 0.01:
         robot_speed = 0
    if abs(robot_angular_vel) <= 0.01:
         robot_angular_vel = 0

    return robot_speed, robot_angular_vel

class SecondOrderDC:
    def __init__(self, dt: float = 0.1):
        # --- 1. Physical Parameters (From your MATLAB code) ---
        Kt = 0.076
        J_motor = 5e-4
        Ra = 1.2
        b = 1.5e-4
        La = 2.5e-3
        self.gear_ratio = 1.0/128
        
        # Load Inertia (Reflected to motor shaft)
        R_wheel = 0.3 # Example radius
        M = 150        # Kg
        N = 1.0 / self.gear_ratio
        Jw = (M / 2) * (R_wheel**2)
        
        # Total Inertia
        J_total = J_motor + (Jw / (N**2))

        # --- 2. Construct Continuous Transfer Function (s-domain) ---
        # G(s) = Kt / (La*J*s^2 + (Ra*J + b*La)s + (Ra*b + Kt^2))
        num = [Kt]
        den = [
            J_total * La, 
            (J_total * Ra) + (b * La), 
            (b * Ra) + (Kt**2)
        ]

        # --- 3. Discretize to Z-domain (The "Magic" Part) ---
        # We use 'bilinear' (Tustin) method, same as c2d(sys, dt, 'tustin') in Matlab
        # This gives us the coefficients for the difference equation
        sys_dt = signal.cont2discrete((num, den), dt, method='bilinear')
        
        # Extract coefficients (numerator b, denominator a)
        # Squeeze removes single-dimensional entries from arrays
        self.b_coeffs = sys_dt[0].squeeze()
        self.a_coeffs = sys_dt[1].squeeze()
        
        # Normalize by a[0] so the formula works cleanly
        # (Standard difference equation form usually requires a[0] = 1)
        a0 = self.a_coeffs[0]
        self.b_coeffs = self.b_coeffs / a0
        self.a_coeffs = self.a_coeffs / a0

        # --- 4. History Buffers (Memory) ---
        # We need to remember the last 2 inputs (u) and last 2 outputs (y)
        # because it is a 2nd order system.
        self.u_hist = [0.0, 0.0] # u[k-1], u[k-2]
        self.y_hist = [0.0, 0.0] # y[k-1], y[k-2]

    def step(self, voltage_input):
        """
        Calculates the next speed sample based on the difference equation:
        y[n] = b0*u[n] + b1*u[n-1] + b2*u[n-2] - a1*y[n-1] - a2*y[n-2]
        """
        
        # 1. Apply the Difference Equation (Direct Form I)
        # Note: a[0] is 1.0, so we ignore it.

        # Inputs (Feedforward)
        term_b = (self.b_coeffs[0] * voltage_input) + \
                 (self.b_coeffs[1] * self.u_hist[0]) + \
                 (self.b_coeffs[2] * self.u_hist[1])
                 
        # Outputs (Feedback)
        term_a = (self.a_coeffs[1] * self.y_hist[0]) + \
                 (self.a_coeffs[2] * self.y_hist[1])

        current_motor_speed = term_b - term_a

        # 2. Update History (Shift the register)
        # Oldest becomes older, newest becomes old
        self.u_hist[1] = self.u_hist[0]
        self.u_hist[0] = voltage_input
        
        self.y_hist[1] = self.y_hist[0]
        self.y_hist[0] = current_motor_speed

        # 3. Convert Motor Speed (rad/s) to Wheel Speed (rad/s)
        wheel_speed = current_motor_speed * self.gear_ratio
        
        return wheel_speed

left_motor_dc = SecondOrderDC()
right_motor_dc = SecondOrderDC()
