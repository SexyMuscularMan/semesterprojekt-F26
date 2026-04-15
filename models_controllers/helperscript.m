clear all;
clc;
close all;
format long g;

%% Models
% Classic model simulation
torque_classic = 0;
[classicModel, classicModel_time, classicModel_theta] = classic_model(torque_classic); % "classicModel" = discrete state space model

% Classic model without tip mass simulation
torque_noTippMass = 0;
[noTippMass, noTippMass_time, noTippMass_theta] = no_tipp_mass(torque_noTippMass); % "noTippMass" = discrete state space model

% Simscape model simulation
simscape_model = sim('SimscapeModel.slx'); % Torque needs to be set in the slx file
data_simscape = simscape_model.simscape_model;
simscapeModel_time = data_simscape.time;
simscapeModel_theta = data_simscape.signals.values;

% Comparing different models
% Plotting pendulum angle for visual comparisons
figure;
plot(classicModel_time, rad2deg(classicModel_theta));
hold on;
plot(noTippMass_time, rad2deg(noTippMass_theta));
hold on;
plot(simscapeModel_time, rad2deg(simscapeModel_theta));
xlabel('time [s]');
ylabel('\theta [deg]');
title('Pendulum Responses');
legend('Classic model','No tipp mass','Simscape model');
grid on;

%% Controller
% Modern control/ pole placement method
pole_placement_results = pole_placement(classicModel);

% Cascade PID
cascade_results = sim('CascadePID.slx');
data_cascade = cascade_results.cascade_results;
cascade_results_time = data_cascade.time;
cascade_results_theta = data_cascade.signals.values;

% Classic PID
classic_results = sim('classicPID.slx');
data_classic = classic_results.classic_results;
classic_results_time = data_classic.time;
classic_results_theta = data_classic.signals.values;

% Parallel PID
% Tuning the gains
classicModel_theta = classicModel(3); % Extracting theta
G = tf(classicModel_theta); % Transfer function
C = pidtune(G, 'PID'); % PID tuning
T_s = 0.01; % Sample time
% Extracting the gain variables
Kp = C.Kp
Ki = C.Ki
Kd = C.Kd
% Simulation
parallel_results = sim('parallelPID.slx');
data_parallel = parallel_results.parallel_results;
parallel_results_time = data_parallel.time;
parallel_results_theta = data_parallel.signals.values;

%% Comparing different controllers 
% Extracting information
pole_placement_info = stepinfo(pole_placement_results.theta, pole_placement_results.time) % Pole placement
cascade_pid_info = stepinfo(cascade_results_theta, cascade_results_time) % Cascade PID
classic_pid_info = stepinfo(classic_results_theta, classic_results_time) % Classic PID
parallel_pid_info = stepinfo(parallel_results_theta, parallel_results_time) % Parallel PID

% Plotting pendulum angle for 10 seconds
figure;
plot(pole_placement_results.time, rad2deg(pole_placement_results.theta));
hold on;
plot(cascade_results_time, rad2deg(cascade_results_theta));
hold on;
plot(classic_results_time, rad2deg(classic_results_theta));
hold on;
plot(parallel_results_time, rad2deg(parallel_results_theta));
xlabel('time [s]');
ylabel('\theta [deg]');
title('Pendulum Responses');
legend('Pole Placement', 'Cascade PID', 'Classic PID', 'Parallel PID');
grid on;

% Plotting pendulum angle for the first 2 seconds
figure;
plot(pole_placement_results.time, rad2deg(pole_placement_results.theta));
hold on;
plot(cascade_results_time, rad2deg(cascade_results_theta));
hold on;
plot(classic_results_time, rad2deg(classic_results_theta));
hold on;
plot(parallel_results_time, rad2deg(parallel_results_theta));
xlim([0 1]);
xlabel('time [s]');
ylabel('\theta [deg]');
title('Pendulum Responses');
legend('Pole Placement', 'Cascade PID', 'Classic PID', 'Parallel PID');
grid on;

%% Root locus analysis
Ts = 0.01;
A = classicModel.A;
B = classicModel.B;
C = classicModel.C;
D = classicModel.D;

% Pole placement
K = pole_placement_results.K; % Extracting Gains
% Open loop system
openLoopSystem = ss(A, B, K, 0, Ts);
G_pole_placement = tf(openLoopSystem);
% Plot
figure;
rlocus(G_pole_placement)
title('Root Locus plot: Pole Placement Controller')
grid on;
xlim([-1.5 1.5]);
ylim([-1 1]);
axis equal;

% Casade PID
% Discrete plant transfer function
G = tf(ss(A,B,C,D,Ts));
z = tf('z', Ts);
% Discrete PID transfer function from Simulink block (Inner loop)
Kp_in = -9.32952485563385;
Ki_in = -53.0199254837238;
Kd_in = -0.386299598154963;
N_in = 194.847386319238;
I_term_in = Kp_in * Ts / (z - 1);
D_term_in = Kd_in * (N_in) / (1 + N_in * Ts/(z - 1));
C_in = Kp_in + I_term_in + D_term_in;
% Discrete PID transfer function from Simulink block (Outer loop)
Kp_out = 0.033376485856261;
Ki_out = 0.00205473985473051;
Kd_out = 0.131363047248811;
N_out = 7.64741103823823;
I_term_out = Ki_out * Ts / (z - 1);
D_term_out = Kd_out * (N_out) / (1 + N_out * Ts/(z - 1));
C_out = Kp_out + I_term_out + D_term_out;
% Open loop system
G_in = feedback(C_in * G(3), 1); % Closed inner closed loop
G_cascade = C_out * G_in; % Cascade open‑loop system with inner loop as plant
% Plot
figure;
rlocus(G_cascade);
title('Root Locus plot: Cascade PID');
grid on;
xlim([-1.5 1.5]);
ylim([-1 1]);
axis equal;

% Classic PID
% figure;
% rlocus(G_classic);
% title('Root Locus plot: Classic PID');
% grid on;
% xlim([-1.5 1.5]);
% ylim([-1 1]);
% axis equal;

% % Parallel PID
% figure;
% rlocus(G_parallel);
% title('Root Locus plot: Parallel PID');
% grid on;
% xlim([-1.5 1.5]);
% ylim([-1 1]);
% axis equal;