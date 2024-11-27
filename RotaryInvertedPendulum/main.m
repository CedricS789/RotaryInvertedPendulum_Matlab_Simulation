clear, close all; clc;
%% Load Saturation and detection parameters
phi_dot_dot_sat = 100; theta_dot_dot_sat = 100; tau_a_sat = inf;
phi_dot_dot_threshold = 1e-10; theta_dot_dot_threshold = 1e-10; tau_a_threshold = 1e-10;

%% Load Physical Parameters
Ip_x2 = 0.02;        % Inertia of pendulum about x2 axis (kg·m²)
Ip_y2 = 0.015;       % Inertia of pendulum about y2 axis (kg·m²)
Ia_z1 = 0.05;        % Inertia of arm about z1 axis (kg·m²)
m_p = 0.5;           % Mass of the pendulum (kg)
l = 1.0;             % Length of the pendulum (m)
r = 0.3;             % Radius or offset length (m)
b_1 = 0.1;           % Damping coefficient for theta (N·m·s)
b_2 = 0.1;           % Damping coefficient (N·m·s)

%% Start Simulink
out = sim('Pendulum');
open_system("Pendulum");

%% Show Plots
%show_positions(out.tout, out.phi, out.theta);

%% Start Animation
animate_pendulum_v2(out.tout, out.phi, out.theta, l, r);
