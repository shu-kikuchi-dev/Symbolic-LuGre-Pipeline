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

%% About Data Filtering
% Only excluding first few seconds of sine waves experiments, and discard
% very first seconds of meso regime (stribeck happens very quickly)
% that contain unstable moment because of the MATLAB solver just start their works.
% We do not have to add any other filters than this, because we completely
% split up 3 phenomenons we want to see, through experimental conditions.
% And we do above filtering with inside of the simulation, means we do not
% establish new program section for filtering. So, we do not store the data
% that should be discard from the moment of data creation.

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
micro_inputs = 'mi_amp-none_w-none_time-none';
meso_inputs = 'me_slope-none_time-none';
macro_inputs = 'ma_amp-none_w-none_time-none';

csv_name = [data_str, '__', explanation, '__', LuGre_params, '__', model_settings, '__', micro_inputs, '__', meso_inputs, '__', macro_inputs, '.csv'];
fig_name = [data_str, '__', explanation, '__', LuGre_params, '__', model_settings, '__', micro_inputs, '__', meso_inputs, '__', macro_inputs, '.pdf'];
% ====================================================================================

% Model Configurations
micro_model = 'LuGre_micro_sinewave';
meso_model = 'Lugre_meso_slowramp';
macro_model = 'LuGre_macro_sinewave';

% Constant LuGre Parameters
sigma0 = 1e5;
sigma1 = sqrt(sigma0);
sigma2 = 0.4;
Fc = 1.0;
Fs = 1.5;
vs = 0.001;

master_table = table();

if ~exist(save_csv_dir, 'dir'), mkdir(save_csv_dir); end
if ~exist(save_fig_dir, 'dir'), mkdir(save_fig_dir); end

fprintf('--- Starting Master Data Collection ---\n');

%% --- Micro Regime: Pre-Sliding, Hysteresis with Sine Waves, micro_model ---
micro_w_list = [0.1, 0.5, 1, 10, 25, 50];
micro_amp_list = [1e-6, 1e-3, 1e-2, 1e-1];

for w_val = micro_w_list
    for amp_val = micro_amp_list
        w = w_val; amp = amp_val; % Push variables to Workspace

        % Dynamic Stop Time: Ensure at least 3 full cycles for z to reach
        % steady state
        stop_time = max(30, (2*pi/w)*3);
        fprintf('Simulating Micro Model: w=%.1f, amp=%.2e, Duration=%.1f\n', w, amp, stop_time);

        simOut = sim(micro_model, 'StopTime', num2str(stop_time));

        ttV = timeseries2timetable(simOut.v_out);
        ttZ = timeseries2timetable(simOut.z_out);
        ttDZ = timeseries2timetable(simOut.dzdt_out);
        ttF = timeseries2timetable(simOut.F_out);

        ts = synchronize(ttV, ttZ, ttDZ, ttF, 'regular', 'linear', 'TimeStep', seconds(0.0001));

        t_one_cycle = 2 * pi / w;
        clean_ts = ts(ts.Time > seconds(t_one_cycle), :);
        v_col = clean_ts{:, 1};
        z_col = clean_ts{:, 2};
        dzdt_col = clean_ts{:, 3};
        F_col = clean_ts{:, 4};
        Source = zeros(size(v_col)); % Source ID: 0

        capsule = table(v_col, z_col, dzdt_col, F_col, Source, ...
            'VariableNames', {'v', 'z', 'dzdt', 'F', 'Source'});
        master_table = [master_table; capsule];
    end
end

%% --- meso Regime: Stribeck Curve, Friction Growing and Dropping, meso_model ---
meso_slope_list = [0.0001, 0.0005, 0.001];

for slope_val = meso_slope_list
    slope = slope_val;

    % Dynamic Stop Time: set the target value as 0.01 and add margin to
    % guarantee the data contain complete plateau
    stop_time = (0.01 / slope_val) + 1;
    fprintf('Simulating meso Model: slope=%.5f, Duration=%.1f\n', slope, stop_time);

    simOut = sim(meso_model, 'StopTime', num2str(stop_time));

    ttV = timeseries2timetable(simOut.v_out);
    ttZ = timeseries2timetable(simOut.z_out);
    ttDZ = timeseries2timetable(simOut.dzdt_out);
    ttF = timeseries2timetable(simOut.F_out);

    ts = synchronize(ttV, ttZ, ttDZ, ttF, 'regular', 'linear', 'TimeStep', seconds(0.0001));

    clean_ts = ts(ts.Time > seconds(0.05), :);
    v_col = clean_ts{:, 1};
    z_col = clean_ts{:, 2};
    dzdt_col = clean_ts{:, 3};
    F_col = clean_ts{:, 4};
    Source = ones(size(v_col)); % Source ID: 1                                                                                                  s(size(v_col)); % Source ID: 0

    capsule = table(v_col, z_col, dzdt_col, F_col, Source, ...
        'VariableNames', {v', 'z', 'dzdt', 'F', 'Source'});
    master_table = [master_table; capsule];
end

%% --- Macro Regime: Viscous Friction, macro_model ---
macro_w_list = [0.1, 1, 5];
macro_amp_list = [1, 1.5, 3];

for w_val = macro_w_list
    for amp_val = macro_amp_list
        w = w_val; amp = amp_val; % Push variables to Workspace

        % Dynamic Stop Time: Ensure at least 3 full cycles for z to reach
        % steady state
        stop_time = max(30, (2*pi/w)*3);
        fprintf('Simulating Macro Model: w=%.1f, amp=%.2e, Duration=%.1f\n', w, amp, stop_time);

        simOut = sim(macro_model, 'StopTime', num2str(stop_time));

        ttV = timeseries2timetable(simOut.v_out);
        ttZ = timeseries2timetable(simOut.z_out);
        ttDZ = timeseries2timetable(simOut.dzdt_out);
        ttF = timeseries2timetable(simOut.F_out);

        ts = synchronize(ttV, ttZ, ttDZ, ttF, 'regular', 'linear', 'TimeStep', seconds(0.0001));

        t_one_cycle = 2 * pi / w;
        clean_ts = ts(ts.Time > seconds(t_one_cycle), :);
        v_col = clean_ts{:, 1};
        z_col = clean_ts{:, 2};
        dzdt_col = clean_ts{:, 3};
        F_col = clean_ts{:, 4};
        Source = 2 *ones(size(v_col)); % Source ID: 2

        capsule = table(v_col, z_col, dzdt_col, F_col, Source, ...
            'VariableNames', {'v', 'z', 'dzdt', 'F', 'Source'});
        master_table = [master_table; capsule];
    end
end

%% --- Ratio Adjusting ---
% I think it is ok to just combine those 3 data equally, 33 % for each.

%% --- Verification Plot ---
% We need to think deeply about how to confirm datasets' reliability and
% quality. Maybe it will take a form of combination of 3D plotting, 2D
% histogram plotting, and machine like counting.

% As a conclusion, this program would be much simpler than former ver 1.0.
% Because the stribeck curve, transient moment detecting filter is not
% needed.

% And, we have to write the each calculation process down here and make a
% independent (from my note) document that has written more extended
% calculation processes, intends of selecting each way and thresholds in
% this program, or something like that.