%% --- Midterm-Presentation Pre-sliding Comparison Graph Generator ---
% This script has intended to generate a graph that compare two models, 
% LuGre and Surrogate Model 1.
% The graph 2 is capturing pre-sliding characteristic, plotted in one graph
% with hold on way. See one with dash line, the other with solid line, so
% that we can see the both models has good accuracy with pre-sliding
% characteristic.

clear; clc;

% --- File Saving Setup ---
date_str = '26-10-02';    % just a 'date' is built-in function, so better to avoid
aspect = 'pre-sliding';
explanation = 'wider-tick-bigger-legend';
model_code = 'LuGre-and-Surr1';
model_settings = 'ode23tb_step-1en4_rel-1en7_abs-1en10';
input_conditions = 'amp-4en6_bias-0_ph-0';
datafactory_conditions = 'params-paper';
save_dir = 'C:\Users\shuki\Projects\work\Symbolic-LuGre-Pipeline\Friction-Model-Evaluation\tmp-outputs\tmp_figs\comparison_for_midterm';
file_name = [date_str, '__', aspect, '__', explanation, '__', model_code, '__', model_settings, '__', input_conditions, '__', datafactory_conditions, '.pdf'];

% Create the folder automatically if it doesn't exist
if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

% --- Parameters ---
model_lugre = 'pre_sliding_Fvsx';
model_surr = 'surrogate_pre_sliding_Fvsx';
t_stop = 10;
t_start_plot = 3;
omega = 70;
period = 2 * pi /omega;

fig = figure('Color', 'w');
hold on;
grid on;

% ======================================
% 1. LuGre (Black Thick Solid Line)
% ======================================
simOut_l = sim(model_lugre, 'StopTime', num2str(t_stop));
idx_l = find(simOut_l.tout >= t_start_plot & simOut_l.tout <=(t_start_plot + period));
plot(simOut_l.x_out.Data(idx_l), simOut_l.F_out.Data(idx_l), ...
    'k-', 'LineWidth', 2.0, 'DisplayName', 'LuGre (Truth)');

% =======================================
% 2. Surrogate Model 1 (Red Dashed Line)
% =======================================
simOut_s = sim(model_surr, 'StopTime', num2str(t_stop));
idx_s = find(simOut_l.tout >= t_start_plot & simOut_l.tout <=(t_start_plot + period));
plot(simOut_s.x_out.Data(idx_s), simOut_s.F_out.Data(idx_s), ...
    'r--', 'LineWidth', 1.5, 'DisplayName', 'Surrogate Model');

% --- Axis Settings ---
xlabel('Displacement x /m');
ylabel('Friction Force F /n');
legend('Location', 'northwest', 'FontSize', 17);
%{
xlim([-5e-6, 5e-6]);
ylim([-0.5, 0.5]);
%}

% Sizing
set(gca, 'FontSize', 16);
set(gca, 'LineWidth', 1.2);
xticks(-4e-6 : 2e-6 : 4e-6);
yticks(-0.4 : 0.2 : 0.4);
box on;


% --- Saving ---
full_save_path = fullfile(save_dir, file_name);
exportgraphics(fig, full_save_path, 'Resolution', 300);
fprintf('Saved: %s\n', full_save_path);