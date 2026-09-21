%% --- LuGre Master Data Factory (version2.0, 2026-09-21) ---
% In this program, we will use only direct velocity input models as data
% generating factory. Previous one, we ve used a Spring-Mass system model
% in addition to the velocity model simulates pre-sliding, hysteresis.
% However, that Stick-Slip motion that Spring-Mass system provides is too
% tricky, chaotic to be treated as data to feed to AI. Because it has
% a long stacking time and break-way happens almost like impulse function.
% The extended logic has been explained at the last part of the note in
% 2026-09-03.

% With this program, we will try to observe mainly three phenomenon.
% 1. Pre-Sliding, Hysteresis: Not breaking-away moments. The bristle make
% complete loops. Observe this through high speed tiny sine waves.
% 2. Stribeck-Curve: Moments contain break-away, bristle growing and
% dropping with proper density, slow speed. With slowly growing ramp.
% 3. Viscous Friction: Observe the moment that the viscous friction has
% dominance. Through huge amplitude sine wave that moves slow.
% The detailed explanation about the each value calculation and entire 
% logic of selecting its ways based on a LuGre model theory has been 
% explained at the note written in 2026-09-18.

% Note that, we will use such well-calculated thresholds and input values
% through this program for guarantees of the reliability and clearness of
% the results is because of our data generation quality or AI settings.
% Extended stating has written in the lower middle, starts from
% "Actually, I am trying...", note of 2026-09-18.

clearvars; clc; close all;

% ====================================================================================
% USER SETTINGS (Directory and Naming)
% ====================================================================================
% FOR MY DESKTOP save_csv_dir = 'D:\shu-kikuchi-projects\MATLAB_project\LuGre_Sim\outputs\tmp_csv_files';
% FOR MY DESKTOP save_fig_dir = 'D:\shu-kikuchi-projects\MATLAB_project\LuGre_Sim\outputs\tmp_figs\MasterData';

% FOR MY LAPTOP
save_csv_dir = 'C:\Users\shuki\Projects\work\Symbolic-LuGre-Pipeline\LuGre_Sim\tmp-outputs\tmp_csv_files';
% FOR MY LAPTOP 
save_fig_dir = 'C:\Users\shuki\Projects\work\Symbolic-LuGre-Pipeline\LuGre_Sim\tmp-outputs\tmp_figs';

data_str = '26-09-21';
explanation = 'first-attempt';
LuGre_params = 'params-paper';
% params-paper: sigma0=1e5, sigma2=0.4, Fc=1.0, Fs=1.5, vs=0.001
model_settings = 'modelsetting-usual';
% modelsetting-usual: ode23tb, step-1en4, rel-1en7, abs-1en10
micro_inputs = 'mi_amp-none_omega-none_time-none';
messo_inputs = 'me_slope-none_time-none';
macro_inputs = 'ma_amp-none_omega-none_time-none';

csv_name = [data_str, '__', explanation, '__', LuGre_params, '__', model_settings, '__', micro_inputs, '__', messo_inputs, '__', macro_inputs, '.csv'];
fig_name = [data_str, '__', explanation, '__', LuGre_params, '__', model_settings, '__', micro_inputs, '__', messo_inputs, '__', macro_inputs, '.pdf'];
% ====================================================================================