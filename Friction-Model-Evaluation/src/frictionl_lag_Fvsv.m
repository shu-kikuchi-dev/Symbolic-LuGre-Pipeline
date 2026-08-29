%% Surrogate Models Frictional Analysis
clear; clc;

% --- Parameters ---
model_name = 'frictional_lag_Fvsv';
t_stop = 30;
t_start_plot = 3;

% Velocity Sine Wave Parameters (Bias > Amplitude to ensure unidirectional)
%Bias = 1.5e-3;
%Amp = 1.0e-3;

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
    w = omegas(i);

    fprintf('Running simulation for omega = %d rad/s...\n', w);

    % Run the simulation
    simOut = sim(model_name, 'StopTime', num2str(t_stop));

    % Extract Data
    t = simOut.tout;
    v = simOut.v_out.Data;
    F = simOut.F_out.Data;

    % Filter for Steady State
    idx = find(t > t_start_plot);
    v_ss = v(idx);
    F_ss = F(idx);

    % Plot F vs v
    plot(v_ss, F_ss, colors(i), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('\\omega = %d rad/s', w));
end

% Finalize Graph
xlabel('Velocity v /(m/s)');
ylabel('Friction Force F /N');
title('Frictional Lag: F vs v Hysteresis (Steady State');
legend('show', 'Location', 'best');

% Adjust axes to see the loops clearly
xlim([0.4e-3, 2.6e-3]);

hold off;
fprintf('All simulations complete.\n');