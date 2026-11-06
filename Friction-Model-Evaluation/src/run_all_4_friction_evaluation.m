%% ========================================================================
%   MASTER FRICTION EVALUATION SUITE: LuGre (Truth) vs. Surrogate Model 2B
%   Solver Engine: ode23tb (TR-BDF2 Stiff Solver)
%   Tolerances: MaxStep = 1e-4, RelTol = 1e-7, AbsTol = 1e-10
%   Author: Shu Kikuchi
% ========================================================================
clearvars; clc; close all;

%% --- 0. GLOBAL SETUP & MODEL DEFINITIONS ---
fprintf('====================================================================\n');
fprintf('   RUNNING 4-PHENOMENON EVALUATION SUITE WITH ode23tb SOLVER       \n');
fprintf('====================================================================\n');

date_and_exp = '26-11-05_second-attempt-with-more-realistic-test4-conditions';

save_dir = fullfile(pwd, 'tmp-outputs', 'model15', date_and_exp);
if ~exist(save_dir, 'dir'), mkdir(save_dir); end

% --- Physical LuGre Parameters (Ground Truth) ---
p.sigma0 = 1e5;
p.sigma1 = sqrt(p.sigma0);
p.sigma2 = 0.4;
p.Fc     = 1.0;
p.Fs     = 1.5;
p.vs     = 0.001;

% --- Surrogate Model 15 Constants (The Champion Model) ---
p.S0 = 1e5;
p.C1 = 0.00042237472;
p.C2 = 2.807613;
p.K  = (p.S0^2) * p.C1; % Precomputed constant: 4.2237e6

% --- Exact Solver Options (User-Specified Thesis Settings) ---
p.opts = odeset('MaxStep', 1e-4, ...
                'RelTol',  1e-7, ...
                'AbsTol',  1e-10);

% Visual Styles
c_lugre = [0, 0.447, 0.741];      % Blue Solid: Ground Truth
c_surr  = [0.850, 0.325, 0.098];    % Red Dashed: Surrogate Model
lw_base = 1.5;

%% ========================================================================
%% TEST 1: PRE-SLIDING HYSTERESIS (F vs. x) via ode23tb
%  Subplot1: Fixes omega=10 rad/s and sweeps A=[1, 5, 14, 30] um
%  Subplot2: Fixes A=4 um and sweeps omega=[0.1, 1, 10, 50, 200, 1000] rad/s
%% ========================================================================
fprintf('\n>>> Running Test 1: Pre-Sliding Hysteresis (ode23tb)...\n');

fig1 = figure('Name', 'Test 1: Pre-Sliding Stress-Test', 'Color', 'w', 'Position', [50, 50, 1100, 450]);

% --- Subplot 1: Amplitude Boundary Sweep (A = 1, 5, 14, 30 um at w = 10 rad/s) ---
subplot(1, 2, 1); hold on; grid on;
amp_sweep = [1e-6, 5e-6, 14e-6, 30e-6];
w_fixed = 10;
t_end_amp = (2*pi/w_fixed) * 3;

for A_val = amp_sweep
    v_fun = @(t) A_val * w_fixed * cos(w_fixed * t);
    x_fun = @(t) A_val * sin(w_fixed * t);

    [t_l, z_l] = solve_kinematic(v_fun, [0, t_end_amp], p, 'lugre');
    [t_s, z_s] = solve_kinematic(v_fun, [0, t_end_amp], p, 'surrogate');

    F_l = compute_force(v_fun(t_l), z_l, p, 'lugre');
    F_s = compute_force(v_fun(t_s), z_s, p, 'surrogate');

    % Keeping only the last 1 period and throwing away everything before it
    idx_l = t_l > (t_end_amp - 2*pi/w_fixed);
    idx_s = t_s > (t_end_amp - 2*pi/w_fixed);

    plot(x_fun(t_l(idx_l)), F_l(idx_l), 'Color', c_lugre, 'LineWidth', 1.0);
    plot(x_fun(t_s(idx_s)), F_s(idx_s), '--', 'Color', c_surr, 'LineWidth', 1.2);
end
title('Amplitude Boundary Sweep (\omega = 10 rad/s)');
xlabel('Displacement x [m]'); ylabel('Friction Force F [N]');
xlim([-3.2e-5, 3.2e-5]); ylim([-1.2, 1.2]);

% --- Subplot 2: Frequency Sweep (Rate-Independence from 0.1 to 1000 rad/s) ---
subplot(1, 2, 2); hold on; grid on;
w_sweep = [0.1, 1, 10, 50, 200, 1000];
A_fixed = 4e-6;

for w_val = w_sweep
    t_end_w = max(4.0, (2*pi/w_val) * 2); % 2 cycles is enough
    v_fun = @(t) A_fixed * w_val * cos(w_val * t);
    x_fun = @(t) A_fixed * sin(w_val * t);

    [t_l, z_l] = solve_kinematic(v_fun, [0, t_end_w], p, 'lugre');
    [t_s, z_s] = solve_kinematic(v_fun, [0, t_end_w], p, 'surrogate');

    F_l = compute_force(v_fun(t_l), z_l, p, 'lugre');
    F_s = compute_force(v_fun(t_s), z_s, p, 'surrogate');

    idx_l = t_l > (t_end_w - 2*pi/w_val);
    idx_s = t_s > (t_end_w - 2*pi/w_val);

    h_l = plot(x_fun(t_l(idx_l)), F_l(idx_l), 'Color', c_lugre, 'LineWidth', 1.0);
    h_s = plot(x_fun(t_s(idx_s)), F_s(idx_s), '--', 'Color', c_surr, 'LineWidth', 1.2);
end
title('Frequency Sweep (0.1 to 1000 rad/s, A = 4 \mu m)');
xlabel('Displacement x [m]'); ylabel('Friction Force F [N]');
xlim([-5e-6, 5e-6]); ylim([-0.6, 0.6]);
legend([h_l, h_s], {'LuGre (Truth)', 'Surrogate Model 15'}, 'Location', 'northwest');

exportgraphics(fig1, fullfile(save_dir, 'Fig1_Presliding_StressTest.pdf'), 'Resolution', 300);
fprintf('   -> Saved: Fig1_Presliding_StressTest.pdf\n');

%% ========================================================================
%% TEST 2: FRICTIONAL LAG (F vs. v) via ode23tb
%  Subplot1: v_bias=1.0 mm/s, A=0.8 mm/s
%  Subplot2: v_bias=1.5 mm/s, A=1.0 mm/s
%  Subplot3: v_bias=5.0 mm/s, A=2.0 mm/s
%% ========================================================================
fprintf('\n>>> Running Test 2: Frictional Lag Multi-Regime...\n');

lag_w_sweep = [0.1, 1, 25, 50, 100];

% 3 Operating Regimes: [Bias, Amplitude]
regimes = [
    1.0e-3, 0.8e-3;   % Regime 1: Stribeck Cliff
    1.5e-3, 1.0e-3;   % Regime 2: Canonical
    5.0e-3, 2.0e-3    % Regime 3: Viscous Dominance
];
regime_titles = {
    'Stribeck Cliff (1.0 \pm 0.8 mm/s)', ...
    'Canonical Region (1.5 \pm 1.0 mm/s)', ...
    'Viscous Region (5.0 \pm 2.0 mm/s)'
};

fig2 = figure('Name', 'Test 2: Frictional Lag Multi-Regime', 'Color', 'w', 'Position', [50, 100, 1200, 400]);

for reg_idx = 1:3
    subplot(1, 3, reg_idx); hold on; grid on;
    vb = regimes(reg_idx, 1);
    va = regimes(reg_idx, 2);
    
    for w_val = lag_w_sweep
        t_end = max(8.0, (2*pi/w_val)*2);
        v_fun = @(t) vb + va * sin(w_val * t);
        
        [t_l, z_l] = solve_kinematic(v_fun, [0, t_end], p, 'lugre');
        [t_s, z_s] = solve_kinematic(v_fun, [0, t_end], p, 'surrogate');
        
        F_l = compute_force(v_fun(t_l), z_l, p, 'lugre');
        F_s = compute_force(v_fun(t_s), z_s, p, 'surrogate');
        
        idx_l = t_l > (t_end - 2*pi/w_val);
        idx_s = t_s > (t_end - 2*pi/w_val);
        
        h_l = plot(v_fun(t_l(idx_l)), F_l(idx_l), 'Color', c_lugre, 'LineWidth', 1.0);
        h_s = plot(v_fun(t_s(idx_s)), F_s(idx_s), '--', 'Color', c_surr, 'LineWidth', 1.2);
    end
    title(regime_titles{reg_idx});
    xlabel('Velocity v [m/s]'); ylabel('Friction Force F [N]');
    if reg_idx == 1, legend([h_l, h_s], {'LuGre', 'Model 15'}, 'Location', 'northeast'); end
end

exportgraphics(fig2, fullfile(save_dir, 'Fig2_FrictionalLag_StressTest.pdf'), 'Resolution', 300);
fprintf('   -> Saved: Fig2_FrictionalLag_StressTest.pdf\n');

%% ========================================================================
%% TEST 3: VARYING BREAKAWAY FORCE (Exact dz/dt = 0 Event Detection)
%% ========================================================================
fprintf('\n>>> Running Test 3: Varying Breakaway across Masses...\n');

rate_list = [0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500];
mass_list = [0.1, 1.0, 10.0];
markers = {'o', 's', '^'};

fig3 = figure('Name', 'Test 3: Varying Breakaway Mass Sweep', 'Color', 'w', 'Position', [100, 100, 750, 500]);
hold on; grid on;

legend_entries = {};
h_plots = [];

for m_idx = 1:length(mass_list)
    m_val = mass_list(m_idx);
    F_brk_l = zeros(size(rate_list));
    F_brk_s = zeros(size(rate_list));
    
    for i = 1:length(rate_list)
        alpha = rate_list(i);
        % because, breakaway would happen roughly by F=Fs, which means t=Fs/a, 
        % when F = at. And 1.5 is extra time, guarantees breakaway
        t_max = max(2.0, (p.Fs * 1.5) / alpha);
        Y0 = [0; 0];
        
        opts_l = odeset(p.opts, 'Events', @(t, Y) breakaway_event(t, Y, p, 'lugre'));
        opts_s = odeset(p.opts, 'Events', @(t, Y) breakaway_event(t, Y, p, 'surrogate'));
        
        [~, Y_l] = ode23tb(@(t, Y) odefun_breakaway(t, Y, alpha, m_val, p, 'lugre'), [0, t_max], Y0, opts_l);
        [~, Y_s] = ode23tb(@(t, Y) odefun_breakaway(t, Y, alpha, m_val, p, 'surrogate'), [0, t_max], Y0, opts_s);
        
        F_brk_l(i) = compute_force(Y_l(end, 1), Y_l(end, 2), p, 'lugre');
        F_brk_s(i) = compute_force(Y_s(end, 1), Y_s(end, 2), p, 'surrogate');
    end
    
    h1 = semilogx(rate_list, F_brk_l, ['-', markers{m_idx}], 'Color', c_lugre, 'LineWidth', 1.2, 'MarkerSize', 6);
    h2 = semilogx(rate_list, F_brk_s, ['--', markers{m_idx}], 'Color', c_surr, 'LineWidth', 1.2, 'MarkerSize', 6);
    
    h_plots = [h_plots, h1, h2];
    legend_entries = [legend_entries, {sprintf('LuGre (m = %.1f kg)', m_val), sprintf('Model 15 (m = %.1f kg)', m_val)}];
end

title('Varying Breakaway Force vs. Force Rate across Masses');
xlabel('Force Rate dF_{ext}/dt [N/s]'); ylabel('Breakaway Force F_{break} [N]');
xlim([0.03, 700]); ylim([0.9, 1.6]);
legend(h_plots, legend_entries, 'Location', 'northeast');

exportgraphics(fig3, fullfile(save_dir, 'Fig3_VaryingBreakaway_MassSweep.pdf'), 'Resolution', 300);
fprintf('   -> Saved: Fig3_VaryingBreakaway_MassSweep.pdf\n');

%% ========================================================================
%% TEST 4: AUTONOMOUS STICK-SLIP (ode23tb Mass-Spring Plant)
%% ========================================================================
fprintf('\n>>> Running Test 4: Stick-Slip Bifurcation Map...\n');

k_list = [2.0, 20.0, 200.0];
v_pull_list = [0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.4];
m_ss = 1.0;
k_markers = {'o', 's', '^', 'd'};

fig4 = figure('Name', 'Test 4: Bifurcation Map', 'Color', 'w', 'Position', [100, 100, 800, 500]);
hold on; grid on;

h_bif = [];
bif_legends = {};

for k_idx = 1:length(k_list)
    k_val = k_list(k_idx);
    ripple_l = zeros(size(v_pull_list));
    ripple_s = zeros(size(v_pull_list));

    for i = 1:length(v_pull_list)
        vp = v_pull_list(i);
        t_sim_end = max(15.0, (1.5 / (k_val * vp)) * 4); % Enough time for 4 stick-slip cycles
        t_span = [0, t_sim_end];
        Y0 = [0; 0; 0];

        [t_l, Y_l] = ode23tb(@(t, Y) odefun_stick_slip(t, Y, vp, m_ss, k_val, p, 'lugre'), t_span, Y0, p.opts);
        [t_s, Y_s] = ode23tb(@(t, Y) odefun_stick_slip(t, Y, vp, m_ss, k_val, p, 'surrogate'), t_span, Y0, p.opts);

        % Calculate velocity ripple in steady-state (last 40% of time)
        idx_ss_l = t_l > (0.6 * t_sim_end);
        idx_ss_s = t_s > (0.6 * t_sim_end);

        ripple_l(i) = max(Y_l(idx_ss_l, 2)) - min(Y_l(idx_ss_l, 2));
        ripple_s(i) = max(Y_s(idx_ss_s, 2)) - min(Y_s(idx_ss_s, 2));
    end

    h1 = semilogx(v_pull_list, ripple_l, ['-', k_markers{k_idx}], 'Color', c_lugre, 'LineWidth', 1.2, 'MarkerSize', 6);
    h2 = semilogx(v_pull_list, ripple_s, ['--', k_markers{k_idx}], 'Color', c_surr, 'LineWidth', 1.2, 'MarkerSize', 6);

    h_bif = [h_bif, h1, h2];
    bif_legends = [bif_legends, {sprintf('LuGre (k = %.1f N/m)', k_val), sprintf('Model 15 (k = %.1f N/m)', k_val)}];
end

title('Stick-Slip Bifurcation Map: Velocity Ripple \Delta v vs. Pulling Speed');
xlabel('Pulling Velocity v_{pull} [m/s]'); ylabel('Velocity Ripple \Delta v = v_{max} - v_{min} [m/s]');
legend(h_bif, bif_legends, 'Location', 'northeast');

exportgraphics(fig4, fullfile(save_dir, 'Fig4_StickSlip_BifurcationMap.pdf'), 'Resolution', 300);
fprintf('   -> Saved: Fig4_StickSlip_BifurcationMap.pdf\n');

fprintf('\n====================================================================\n');
fprintf('   SUCCESS: ALL 4 BENCHMARKS SIMULATED VIA ode23tb & EXPORTED!      \n');
fprintf('   Folder: %s\n', save_dir);
fprintf('====================================================================\n');


%% ========================================================================
%% ODE EQUATION DEFINITIONS FOR ode23tb
%% ========================================================================

% --- Kinematic Solver for Test 1 & Test 2 ---
function [t_out, z_out] = solve_kinematic(v_fun, tspan, p, model_type)
    if strcmp(model_type, 'lugre')
        ode = @(t, z) v_fun(t) - (p.sigma0 * abs(v_fun(t)) / (p.Fc + (p.Fs - p.Fc)*exp(-(v_fun(t)/p.vs)^2))) * z;
    else
        % MODEL 15:
        ode = @(t, z) v_fun(t) - abs(v_fun(t)) * (p.S0 * z - ((p.S0 * z) / (p.K * (v_fun(t)^2) + p.C2)));
    end
    [t_out, z_out] = ode23tb(ode, tspan, 0, p.opts);
end

% --- Breakaway ODE for Test 3: States Y = [v; z] ---
function dY = odefun_breakaway(t, Y, alpha, m, p, model_type)
    v = Y(1);
    z = Y(2);
    
    if strcmp(model_type, 'lugre')
        gv = p.Fc + (p.Fs - p.Fc) * exp(-(v / p.vs)^2);
        dz = v - (p.sigma0 * abs(v) / gv) * z;
    else
        % MODEL 15:
        s0_z = p.S0 * z;
        dz = v - abs(v) * (s0_z - (s0_z / (p.K * v^2 + p.C2)));
    end
    
    F_fric = p.sigma0 * z + p.sigma1 * dz + p.sigma2 * v;
    F_ext = alpha * t;
    
    dv = (F_ext - F_fric) / m;
    dY = [dv; dz];
end

% --- Breakaway Event Detection Function ---
function [value, isterminal, direction] = breakaway_event(t, Y, p, model_type)
    % Extract states
    v = Y(1);
    z = Y(2);

    % Calculate exact dz/dt
    if strcmp(model_type, 'lugre')
        gv = p.Fc + (p.Fs - p.Fc) * exp(-(v / p.vs)^2);
        dz = v - (p.sigma0 * abs(v) / gv) * z;
    else
        % Surrogate Model 15
        s0_z = p.S0 * z;
        dz = v - abs(v) * (s0_z - (s0_z / (p.K * v^2 + p.C2)));
    end

    % Add this inside breakaway_event right before 'value = dz;':
    if t < 1e-5
        value = 1.0; % Force non-zero during initial start
    else
        value = dz;  % Detect dz/dt = 0 at breakaway
    end
 
    isterminal = 1;  % Stop simulation immediately at breakaway
    direction  = -1; % ONLY trigger when dz/dt crosses from positive to negative!
end

% --- Stick-Slip ODE for Test 4: States Y = [x; v; z] ---
function dY = odefun_stick_slip(t, Y, v_pull, m, k, p, model_type)
    x = Y(1);
    v = Y(2);
    z = Y(3);
    
    if strcmp(model_type, 'lugre')
        gv = p.Fc + (p.Fs - p.Fc) * exp(-(v / p.vs)^2);
        dz = v - (p.sigma0 * abs(v) / gv) * z;
    else
        % MODEL 15:
        s0_z = p.S0 * z;
        dz = v - abs(v) * (s0_z - (s0_z / (p.K * v^2 + p.C2)));
    end
    
    F_fric = p.sigma0 * z + p.sigma1 * dz + p.sigma2 * v;
    F_spring = k * (v_pull * t - x);
    
    dx = v;
    dv = (F_spring - F_fric) / m;
    dY = [dx; dv; dz];
end

% --- Algebraic Force Computation ---
function F = compute_force(v, z, p, model_type)
    if strcmp(model_type, 'lugre')
        gv = p.Fc + (p.Fs - p.Fc) * exp(-(v ./ p.vs).^2);
        dz = v - (p.sigma0 .* abs(v) ./ gv) .* z;
    else
        % MODEL 15 (Vectorized):
        s0_z = p.S0 .* z;
        dz = v - abs(v) .* (s0_z - (s0_z ./ (p.K .* (v.^2) + p.C2)));
    end
    F = p.sigma0 .* z + p.sigma1 .* dz + p.sigma2 .* v;
end