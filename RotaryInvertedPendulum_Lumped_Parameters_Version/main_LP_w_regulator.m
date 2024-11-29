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

P1 = - 0.7065;
P2 = - 1.44;
P3 = 0.05622;
P4 = 0.008979;
P5 = 0.00232;
P6 = 1;

%% Compute state space representation based on lumped parameters

A_upper = [0,  0,                  1,       0;
           0,  0,                  0,       1;
           0,  0,             -P2/P1,       0;
           0, P6/P4, (P3*P2)/(P4*P1),  -P5/P4];

B_upper = [0;
           0;
           1/P1;
           -P3/(P1*P4)];

% % A and B matrices for lower position (shouldn't be needed as the regulation will be done in the upper position)
% 
% % A_lower = [0,  0,                    1,       0;
% %            0,  0,                    0,       1;
% %            0,  0,               -P2/P1,       0;
% %            0, -P6/P4, -(P3*P2)/(P4*P1),  -P5/P4];
% % 
% % B_lower = [0;
% %            0;
% %            1/P1;
% %            P3/(P1*P4)];

% C = [1 1 0 0];

C = [1 1 1 1];

D = 0;

% Save state space representation to be used in the Simulink
assignin('base', 'A_upper', A_upper);
assignin('base', 'B_upper', B_upper);
assignin('base', 'C', C);
assignin('base', 'D', D);


%% Compute the gain thanks to the LQR

Q = [175 0 0 0;0 110 0 0; 0 0 125 0; 0 0 0 40] ;
R = 12 ;
K = lqr(A_upper,B_upper,Q,R);

% Save the gain to be used in the Simulink
assignin('base', 'K', K);

% Open Simulink file 
open_system('Pendulum_LP_linear_w_reg');

% Launch simulation
out = sim('Pendulum_LP_linear_w_reg');

%% Récupération des données des blocs To Workspace
% Remplacez 'variable1' et 'variable2' par les noms des variables enregistrées dans vos blocs To Workspace
t = out.tout;
phi = out.phi;
phi_dot = out.phi_dot;
phi_ref = out.phi_ref;
phi_ref = repmat(phi_ref,size(phi_dot));
theta = out.theta;
theta_dot = out.theta_dot;
theta_ref = out.theta_ref;
theta_ref = repmat(theta_ref,size(phi_dot));

% Création d'une nouvelle figure
figure;

% Premier subplot : phi et phi_ref
subplot(4, 1, 1);
plot(t, phi, 'b', 'LineWidth', 1.5); % Courbe pour phi en bleu
hold on;
plot(t, phi_ref, 'r--', 'LineWidth', 1.5); % Courbe pour phi_ref en rouge pointillée
hold off;
xlabel('Time (s)');
ylabel('\phi, \phi_{ref}');
legend('\phi', '\phi_{ref}');
title('Phi and Phi Reference');
grid on;

% Deuxième subplot : theta et theta_ref
subplot(4, 1, 2);
plot(t, theta, 'g', 'LineWidth', 1.5); % Courbe pour theta en vert
hold on;
plot(t, theta_ref, 'm--', 'LineWidth', 1.5); % Courbe pour theta_ref en magenta pointillée
hold off;
xlabel('Time (s)');
ylabel('\theta, \theta_{ref}');
legend('\theta', '\theta_{ref}');
title('Theta and Theta Reference');
grid on;

% Troisième subplot : theta_dot
subplot(4, 1, 3);
plot(t, theta_dot, 'c', 'LineWidth', 1.5); % Courbe pour theta_dot en cyan
xlabel('Time (s)');
ylabel('\theta_{dot}');
title('Theta Dot');
grid on;

% Quatrième subplot : phi_dot
subplot(4, 1, 4);
plot(t, phi_dot, 'k', 'LineWidth', 1.5); % Courbe pour phi_dot en noir
xlabel('Time (s)');
ylabel('\phi_{dot}');
title('Phi Dot');
grid on;

% Ajuster les espacements entre les subplots
sgtitle('Comparaison des données');


%% Start Animation
r = 9.81 * (P3/P6);
l = r;
animate_pendulum_v2(out.tout, out.phi, out.theta, l/4, r/4);
