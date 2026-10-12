%% --- LuGre Master Data Factory (version2.0, 2026-09-21) ---
% In this program, we will use only direct velocity input models as data
% generating factory. With previous one, we ve used a Spring-Mass system model
% in addition to the velocity model simulates pre-sliding, hysteresis.
% However, that Stick-Slip motion that Spring-Mass system provides is too
% tricky, rapid to be treated as data to feed to AI. Because it has
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
save_fig_dir = 'C:\Users\shuki\Projects\work\Symbolic-LuGre-Pipeline\LuGre_Sim\tmp-outputs\tmp_figs\26-10-12_only-velocity';

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
fig_name = [data_str, '__', explanation, '__', LuGre_params, '__', model_settings, '__', micro_inputs, '__', meso_inputs, '__', macro_inputs];
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

scale = 1e5;

master_table = table();

if ~exist(save_csv_dir, 'dir'), mkdir(save_csv_dir); end
if ~exist(save_fig_dir, 'dir'), mkdir(save_fig_dir); end

fprintf('--- Starting Master Data Collection ---\n');

%% --- Micro Regime: Pre-Sliding, Hysteresis with Sine Waves, micro_model ---
micro_w_list = [10, 25, 50];
micro_amp_list = [1e-5, 5e-5, 1.5e-4, 3.75e-4];

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

        capsule = table(v_col*scale, z_col*scale, dzdt_col*scale, F_col, Source, ...
            'VariableNames', {'v_norm', 'z_norm', 'dzdt_norm', 'F', 'Source'});
        master_table = [master_table; capsule];
    end
end

%% --- Meso Regime: Stribeck Curve, Friction Growing and Dropping, meso_model ---
meso_slope_list = [-0.001, -0.0005, -0.0001, 0.0001, 0.0005, 0.001];

for slope_val = meso_slope_list
    slope = slope_val;

    % Dynamic Stop Time: set the target value as 0.01 and add margin to
    % guarantee the data contain complete plateau
    stop_time = (0.01 / abs(slope_val)) + 1;
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

    capsule = table(v_col*scale, z_col*scale, dzdt_col*scale, F_col, Source, ...
        'VariableNames', {'v_norm', 'z_norm', 'dzdt_norm', 'F', 'Source'});
    master_table = [master_table; capsule];
end

%% --- Macro Regime: Viscous Friction, macro_model ---
macro_w_list = [1, 2, 5];
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

        capsule = table(v_col*scale, z_col*scale, dzdt_col*scale, F_col, Source, ...
            'VariableNames', {'v_norm', 'z_norm', 'dzdt_norm', 'F', 'Source'});
        master_table = [master_table; capsule];
    end
end

%% --- Ratio Adjusting ---
% I think it is ok to just combine those 3 data equally, 33 % for each.
fprintf('Refining data to exactly 200 000 rows...\n');

% Define Target Row Counts
target_total = 200000;
n_micro_target = round(target_total / 3);
n_meso_target = round(target_total / 3);
n_macro_target = target_total - n_micro_target - n_meso_target;

% Separate Row Indices
idx_mi = find(master_table.Source == 0);    % Micro
idx_me = find(master_table.Source == 1);    % Meso
idx_ma = find(master_table.Source == 2);    % Macro

% Extraction
fprintf('   - Extracting data...\n"');
keep_mi = idx_mi(round(linspace(1, length(idx_mi), n_micro_target)));
keep_me = idx_me(round(linspace(1, length(idx_me), n_meso_target)));
keep_ma = idx_ma(round(linspace(1, length(idx_ma), n_macro_target)));

% Combine and sort
final_idx = sort([keep_mi; keep_me; keep_ma]);
final_table = master_table(final_idx, :);

%% --- Quality Audit ---
% Unscale v_norm for auditing.
v_phys = final_table.v_norm / scale;

v_crit_limit = 0.005; % 5*vs for stribeck zone, 10*vs for a start of complete plateau.
n_total = size(final_table, 1);
n_crit = sum(abs(v_phys) < v_crit_limit);
pct_crit = (n_crit / n_total) * 100;

fprintf('\n--- Dataset Quality Audit ---\n');
fprintf('Total Dataset Size: %d rows\n', n_total);
fprintf('Critical Zone (|v| < %.3f m/s): %d rows (%.1f%%)\n', v_crit_limit, n_crit, pct_crit);

%% --- CSV Export (Shuffled) ---
csv_path = fullfile(save_csv_dir, csv_name);
shuffled_table = final_table(randperm(size(final_table, 1)), :);
writetable(shuffled_table, csv_path);
fprintf('Dataset saved successfully as: %s\n', csv_name);

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

% 2D Velocity Distribution Check
fig_hist = figure('Name', 'Velocity Distribution Check', 'Color', 'w');
histogram(v_phys, 150);
hold on;
line([v_crit_limit, v_crit_limit], ylim, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 2);
line([-v_crit_limit, -v_crit_limit], ylim, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 2);
title('Velocity Distribution (Red dashed = STribeck Zone)');
xlabel('Velocity v /(v/m)');
ylabel('Sammple Count');
grid on;
xlim([-0.02, 0.02]); % Zoomed in to verify the Stribeck zone bridge;

hist_pdf_name = fullfile(save_fig_dir, [data_str, '__Histogram_v2.pdf']);
exportgraphics(fig_hist, hist_pdf_name, 'Resolution', 300);
fprintf('Histogram saved as: %s\n', hist_pdf_name);

% 3D Phase Portrait (F vs v_phys vs z_norm)
fig_3d = figure('Name', '3-Regime Phase Portrait', 'Color', 'w', 'Position', [100, 100, 900, 700]);
hold on; grid on;

% Sample 3000 points from each regime for clean plotting
sample_n = 3000;
idx_p0 = find(final_table.Source == 0); idx_p0 = idx_p0(randperm(length(idx_p0), sample_n));
idx_p1 = find(final_table.Source == 1); idx_p1 = idx_p1(randperm(length(idx_p1), sample_n));
idx_p2 = find(final_table.Source == 2); idx_p2 = idx_p2(randperm(length(idx_p2), sample_n));

% Blue: Micro
scatter3(v_phys(idx_p0), final_table.z_norm(idx_p0), final_table.F(idx_p0), 6, [0 0.447 0.741], 'filled');
% Green: Meso (Ramp)
scatter3(v_phys(idx_p1), final_table.z_norm(idx_p1), final_table.F(idx_p1), 6, [0.466 0.674 0.188], 'filled');
% Orange: Macro
scatter3(v_phys(idx_p2), final_table.z_norm(idx_p2), final_table.F(idx_p2), 6, [0.850 0.325 0.098], 'filled');

title('3-Regime Decoupled State-Space Manifold');
xlabel('Velocity v /(m/s)');
ylabel('Normalized State z_{norm}');
zlabel('Friction Force F /N');
kegend({'Miscro (Pre-sliding)', 'Meso (Stribeck Ramp)', 'Macro (Viscous Sine)'}, 'Location', 'best');
view(45, 30);

fig3d_name = fullfile(save_fig_dir, [fig_name, '.fig']);
savefig(fig_final, fig_path);
fprintf('Figure saved as: %s.fig\n', fig_name);

fprintf('   --- All prpcesses have finished! ---');