%% Surrogate Models Pre-Sliding Analysis
clear; clc;

% --- File Saving Setup ---
date_str = '26-08-30';    % just a 'date' is built-in function, so better to avoid
aspect = 'presliding';
model_code = 'LuGre';
model_settings = 'ode23tb_maxstepsize-1en4_relativetorelance-1en7_absolutetorelance-1en10';
input_conditions = 'amplitude-4en6_bias-0_phase-0_omega-40';
datafactory_conditions = 'params-paper';
save_dir = 'C:\Users\shuki\Projects\work\Symbolic-LuGre-Pipeline\Friction-Model-Evaluation\outputs\figs\LuGre';
file_name = [date_str, '__', aspect, '__', model_code, '__', model_settings, '__', input_conditions, '__', datafactory_conditions, '.pdf'];

% Create the folder automatically if it doesn't exist
if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

% --- Parameters ---
model_name = 'pre_sliding_Fvsx';
t_stop = 10;
omega = 40;

% --- Figure Initialization ---
fig = figure;
hold on;
grid on;

% --- Run the Simulation ---
fprintf('Running simulation...\n')
simOut = sim(model_name, 'StopTime', num2str(t_stop));

% Extract Data
t = simOut.tout;
x = simOut.x_out.Data;
F = simOut.F_out.Data;
idx = find(t > 3);

% Plot F vs x
plot(x(idx), F(idx));
xlabel('Displacement x /m');
ylabel('Friction Force F /N');
title('Pre-Sliding: F vs x Hysteresis');

% Adjust axes to see the loops clearly
xlim([-5e-6, 5e-6]);
ylim([-0.5, 0.5])

hold off;

% --- Save the fig ---
full_save_path = fullfile(save_dir, file_name);
exportgraphics(fig, full_save_path, 'Resolution', 300);

fprintf('Graph saved to: %s\n', full_save_path);
fprintf('A simulation complete.\n');