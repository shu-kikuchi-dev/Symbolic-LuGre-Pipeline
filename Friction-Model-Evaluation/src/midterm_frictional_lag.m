%% --- Midterm-Presentation Frictional Lag Comparison Graph Generator ---
% This script has intended to generate a graph that compare two models, 
% LuGre and Surrogate Model 1.
% The graph is capturing frictional-lag characteristic, plotted with twin y
% axis due to both models scale difference.

clear; clc;

% --- File Saving Setup ---
date_str = '26-10-02';    % just a 'date' is built-in function, so better to avoid
aspect = 'frictional-lag';
explanation = 'Second-attempt-with-single-w';
model_code = 'LuGre-and-Surr1';
model_settings = 'ode23tb_step-1en4_rel-1en7_abs-1en10';
input_conditions = 'amp-1en3_bias-1p5en3_ph-0';
datafactory_conditions = 'params-paper';
save_dir = 'C:\Users\shuki\Projects\work\Symbolic-LuGre-Pipeline\Friction-Model-Evaluation\tmp-outputs\tmp_figs\comparison_for_midterm';
file_name = [date_str, '__', aspect, '__', model_code, '__', model_settings, '__', input_conditions, '__', datafactory_conditions, '.pdf'];

% Create the folder automatically if it doesn't exist
if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

% --- Parameters ---
model_lugre = 'frictional_lag_Fvsv';
model_surr = 'surrogate_frictional_lag_Fvsv';
t_stop = 30;
t_start_plot = 3;
omegas = [1, 10, 25];
colors = {'b', 'r', 'g'};

fig = figure('Color', 'w');
grid on;

% ===========================
% 1. Left side axis: LuGre
% ===========================
yyaxis left
hold on;
%{
for i = 1:length(omegas)
    w = omegas(i);
    simOut = sim(model_lugre, 'StopTime', num2str(t_stop));
    idx = find(simOut.tout > t_start_plot);
    plot(simOut.v_out.Data(idx), simOut.F_out.Data(idx), ...
        'Color', colors{i}, 'LineStyle', '-', 'LineWidth', 1.5, ...
        'DisplayName', sprintf('LuGre (\\omega = %d)', w));
end
%}
w = omegas(3);
simOut = sim(model_lugre, 'StopTime', num2str(t_stop));
idx = find(simOut.tout > t_start_plot);
plot(simOut.v_out.Data(idx), simOut.F_out.Data(idx), ...
    'Color', colors{1}, 'LineStyle', '-', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('LuGre (\\omega = %d)', w));

ylabel('Friction Force (LuGre) /N');
ylim([1, 1.4]); % Frictional Force fluctuate range of LuGre

% ===========================
% 2. Right side axis: Surrogate Model 1
% ===========================
yyaxis right
hold on;
%{
for i = 1:length(omegas)
    w = omegas(i);
    simOut = sim(model_surr, 'StopTime', num2str(t_stop));
    idx = find(simOut.tout > t_start_plot);
    plot(simOut.v_out.Data(idx), simOut.F_out.Data(idx), ...
        'Color', colors{i}, 'LineStyle', '--', 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Surr1 (\\omega = %d)', w));
end
%}
w = omegas(3);
simOut = sim(model_surr, 'StopTime', num2str(t_stop));
idx = find(simOut.tout > t_start_plot);
plot(simOut.v_out.Data(idx), simOut.F_out.Data(idx), ...
    'Color', colors{2}, 'LineStyle', '--', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Surr1 (\\omega = %d)', w));

ylabel('Friction Force (Surrogate) /N');
ylim([1.002, 1.012]); % Frictional Force fluctuate range of Surrogate Modle

% --- Common Configurations ---
xlabel('Velocity /(m/s)');
title('Frictional Lag: F vs v Hysteresis (Dual-Axis Comparison)');

% --- Saving ---
full_save_path = fullfile(save_dir, file_name);
exportgraphics(fig, full_save_path, 'Resolution', 300);
fprintf('Saved: %s\n', full_save_path);