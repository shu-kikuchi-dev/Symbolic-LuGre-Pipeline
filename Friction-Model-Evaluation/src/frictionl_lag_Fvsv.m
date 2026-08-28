%% Surrogate Models Frictional Analysis
clear; clc;

% --- Parameters ---
model_name = 'frictional_lag_Fvsv';
t_stop = 30;
t_start_plot = 3;

% Velocity Sine Wave Parameters (Bias > Amplitude to ensure unidirectional)
Bias = 1.5e-3;
Amp = 1.0e-3;

% Frequencies
omegas = [1, 10, 25];
colors = ['b', 'r', 'g'];

% Initialize Figure
figure('Color', 'w');
hold on;
grid on;

% --- Loop Through Frequencies ---
for i = 1:length(omegas)
    % Update the workspace variable that Simulink uses