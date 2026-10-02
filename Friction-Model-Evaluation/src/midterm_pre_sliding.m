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
model_code = 'LuGre-and-Surr1';
model_settings = 'ode23tb_step-1en4_rel-1en7_abs-1en10';
input_conditions = 'amp-4en6_bias-0_ph-0_w-40';
datafactory_conditions = 'params-paper';
save_dir = 'C:\Users\shuki\Projects\work\Symbolic-LuGre-Pipeline\Friction-Model-Evaluation\outputs\figs\comparison_for_midterm';
file_name = [date_str, '__', aspect, '__', model_code, '__', model_settings, '__', input_conditions, '__', datafactory_conditions, '.pdf'];