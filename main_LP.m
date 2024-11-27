clear, close all; clc;
%% Load Saturation and detection parameters
phi_dot_dot_sat = 100; theta_dot_dot_sat = 100; tau_a_sat = inf;
phi_dot_dot_threshold = 1e-10; theta_dot_dot_threshold = 1e-10; tau_a_threshold = 1e-10;

%% Load Lumped Parameters
% Ip_x2  - Inertia of pendulum about x2 axis (kg·m²)
% Ip_y2  - Inertia of pendulum about y2 axis (kg·m²)
% Ia_z1  - Inertia of arm about z1 axis (kg·m²)
% m_p    - Mass of the pendulum (kg)
% l      - Length of the pendulum (m)
% r      - Radius or offset length (m)
% b_1    - Damping coefficient for theta (N·m·s)
% b_2    - Damping coefficient (N·m·s)

% P1 = Ia_z1 + mp * r^2
% P2 = b1
% P3 = mp * l * r
% P4 = 2 * Ip_x2
% P5 = 2 * b2
% P6 = mp * g * l

P1 = 0.7065;
P2 = 1.44;
P3 = 0.05622;
P4 = 0.008979;
P5 = 0.00232;
P6 = 1;


%% Start Simulink
out = sim('Pendulum_LP');
open_system("Pendulum_LP");

%% Show Plots
%show_positions(out.tout, out.phi, out.theta);

%% Start Animation
r = 9.81 * (P3/P6);
l = r;
animate_pendulum_v2(out.tout, out.phi, out.theta, l/4, r/4);
