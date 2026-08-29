%% Surrogate Models Frictional Analysis
clear; clc;

% --- File Saving Setup ---
date_str = '26-08-29';    % just a 'date' is built-in function, so better to avoid
model_code = 'LuGre';
model_settings = 'ode23tb_maxstepsize-1en4_relativetorelance-1en7_absolutetorelance-1en10';
input_conditions = 'amplitude-1en3_bias-1p5en3_phase-0';
datafactory_conditions = 'params-paper';
save_dir = 'C:\Users\shuki\Projects\work\Symbolic-LuGre-Pipeline\Friction-Model-Evaluation\figs\LuGre';
file_name = [date_str, '__', model_code, '__', model_settings, '__', input_conditions, '__', datafactory_conditions, '.pdf'];

% Create the folder automatically if it doesn't exist
if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

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
fig = figure('Color', 'w');
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
title('Frictional Lag: F vs v Hysteresis (Steady State)');
legend('show', 'Location', 'best');

% Adjust axes to see the loops clearly
xlim([0.4e-3, 2.6e-3]);

hold off;

% --- Save the fig ---
full_save_path = fullfile(save_dir, file_name);
exportgraphics(fig, full_save_path, 'Resolution', 300);

fprintf('Graph saved to: %s\n', full_save_path);
fprintf('All simulations complete.\n');