%% Surrogate Models Pre-Sliding Analysis
clear; clc;

% --- File Saving Setup ---
date_str = '26-08-29';    % just a 'date' is built-in function, so better to avoid
aspect = 'presliding';
model_code = 'LuGre';
model_settings = 'ode23tb_maxstepsize-1en4_relativetorelance-1en7_absolutetorelance-1en10';
input_conditions = 'amplitude-1en3_bias-1p5en3_phase-0';
datafactory_conditions = 'params-paper';
save_dir = 'C:\Users\shuki\Projects\work\Symbolic-LuGre-Pipeline\Friction-Model-Evaluation\figs\LuGre';
file_name = [date_str, '__', aspect, '__', model_code, '__', model_settings, '__', input_conditions, '__', datafactory_conditions, '.pdf'];
