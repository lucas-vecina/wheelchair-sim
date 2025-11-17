clc
clear
close all

%%% Input (Polar System)
t_inp = [0,   0.5,   2.0,     2.5,     3,   3.5,   5,    5.5,      6,    7.5,   8]'; % s
rho =   [0,     1,     1,     0  ,  0.75,  0.75,   0,      0,   0.25,   0.25,   0]'; % Normalized Amplitude (0-1)
theta = [0,  pi/2,  pi/2,     0  ,    pi,    pi,   0,      0,  -pi/6,  -pi/6,   0]'; % Angle rad

simin = timeseries([rho, theta], t_inp);

%%% Joystick Model
lim_angles = [-1, 1];   % deg
lim_voltage = [0, 5];     % Volts

%%% Noise Model
noise_power = 0.1;
noise_gain = 0.001;
noise_bias = 0;
noise_frequency = 1000; % Hz

noise_power_2 = 0.1;
noise_gain_2 = 0.01;
noise_bias_2 = 0;
noise_frequency_2 = 10000;%4; % Hz

%%% ADC Model
bits = 10;
lim_adc = [0, 2^bits-1]; % Digital voltage
adc_gain = (lim_adc(2)+1)/lim_voltage(2);

%%% Robot Model
L = 0.7; % Width (m)
R = 0.3; % Wheel Radius (m)
initial_pose = 0; % rad

forward_kinematics = [
  [R/2, R/2];
  [-R/L, R/L]
];

inverse_kinematics = [
  [1/R, -L/(2*R)];
  [1/R, L/(2*R)]
];

%%% PWM Converter Model
lim_pwm = [0, 1]; % Duty Cycle

%%% H-Bridge Driver Model
vin = 24; % V

%%% PMDC model
Kt = 0.076;
J = 5e-4;
Ra = 1.2;
b = 1.5e-4;
La = 2.5e-3;
gear_ratio = 1/190;

M = 150; % Kg
cf = 0.05; % Friction coef
g = 9.81;  % Gravity
N = 1/gear_ratio;

Jw = M/2*R^2;
J1 = J + Jw/N^2;

Fm = M*g*cf/2;
Tm = Fm*R/N;

s = tf('s');
num_motor_tf = Kt;
den_motor_tf = [J*La, J*Ra+b*La, b*Ra+Kt^2];
[A,B,C,D] = tf2ss(num_motor_tf, den_motor_tf);
sys = ss(A,B,C,D);

%%% Parameters Estimation
step_resp = stepinfo(sys*vin*gear_ratio*R);
motor_max_speed = step_resp.SettlingMax;
lim_motor_speed = [-motor_max_speed, motor_max_speed]; % m/s
kinematic_resp = forward_kinematics*[motor_max_speed;motor_max_speed];
lim_robot_speed = [-kinematic_resp(1), kinematic_resp(1)]; % m/s
kinematic_resp = forward_kinematics*[-motor_max_speed;motor_max_speed];
lim_robot_angular_velocity = [-kinematic_resp(2), kinematic_resp(2)]; % rad/s

%%% Coeficients
c1 = polyfit(lim_angles, lim_voltage, 1);
c2 = polyfit(lim_adc, lim_robot_speed, 1);
c3 = polyfit(lim_adc, lim_robot_angular_velocity, 1);
c4 = polyfit([0, lim_motor_speed(2)], lim_pwm, 1);

out = sim("system_control.slx", t_inp(end)+1);

odometry = [
    out.tout, ...
    out.odometry.signals(1).values, ...
    out.odometry.signals(2).values, ...
    out.odometry.signals(3).values
];
writematrix(odometry, "data/odometry.dat")