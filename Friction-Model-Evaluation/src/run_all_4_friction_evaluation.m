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

save_dir = fullfile(pwd, '..', 'tmp-outputs', 'model15');
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
%% ========================================================================
fprintf('\n>>> Running Test 1: Pre-Sliding Hysteresis (ode23tb)...\n');

w_canon = [1, 10, 25];
A_canon = 4e-6; % 4 um (Well below delta = 15 um)

fig1 = figure('Name', 'Test 1: Pre-Sliding Canonical', 'Color', 'w', 'Position', [100, 100, 700, 500]);
hold on; grid on;

for w_val = w_canon
    t_end = max(10, (2*pi/w_val)*4);
    
    % Kinematic handles
    v_fun = @(t) A_canon * w_val * cos(w_val * t);
    x_fun = @(t) A_canon * sin(w_val * t);
    
    % Simulate state z(t) via ode23tb
    [t_l, z_l] = solve_kinematic(v_fun, [0, t_end], p, 'lugre');
    [t_s, z_s] = solve_kinematic(v_fun, [0, t_end], p, 'surrogate');
    
    % Compute Force
    F_l = compute_force(v_fun(t_l), z_l, p, 'lugre');
    F_s = compute_force(v_fun(t_s), z_s, p, 'surrogate');
    
    % Extract last steady-state cycle
    idx_l = t_l > (t_end - 2*pi/w_val);
    idx_s = t_s > (t_end - 2*pi/w_val);
    
    plot(x_fun(t_l(idx_l)), F_l(idx_l), 'Color', c_lugre, 'LineWidth', lw_base);
    plot(x_fun(t_s(idx_s)), F_s(idx_s), '--', 'Color', c_surr, 'LineWidth', lw_base);
end
title('Pre-Sliding Hysteresis: F vs. x (A = 4 \mu m) [ode23tb]');
xlabel('Displacement x [m]'); ylabel('Friction Force F [N]');
xlim([-5e-6, 5e-6]); ylim([-0.5, 0.5]);
legend({'LuGre (Truth)', 'Surrogate Model 15'}, 'Location', 'northwest');

exportgraphics(fig1, fullfile(save_dir, 'Fig1_Presliding_Canonical.pdf'), 'Resolution', 300);
fprintf('   -> Saved: Fig1_Presliding_Canonical.pdf\n');

%% ========================================================================
%% TEST 2: FRICTIONAL LAG (F vs. v) via ode23tb
%% ========================================================================
fprintf('\n>>> Running Test 2: Frictional Lag (ode23tb)...\n');

lag_w_list = [1, 10, 25];
v_bias = 1.5e-3;
v_amp  = 1.0e-3;

fig2 = figure('Name', 'Test 2: Frictional Lag Canonical', 'Color', 'w', 'Position', [150, 150, 750, 500]);
hold on; grid on;

for w_val = lag_w_list
    t_end = max(15, (2*pi/w_val)*4);
    v_fun = @(t) v_bias + v_amp * sin(w_val * t);
    
    [t_l, z_l] = solve_kinematic(v_fun, [0, t_end], p, 'lugre');
    [t_s, z_s] = solve_kinematic(v_fun, [0, t_end], p, 'surrogate');
    
    F_l = compute_force(v_fun(t_l), z_l, p, 'lugre');
    F_s = compute_force(v_fun(t_s), z_s, p, 'surrogate');
    
    idx_l = t_l > (t_end - 2*pi/w_val);
    idx_s = t_s > (t_end - 2*pi/w_val);
    
    h_l = plot(v_fun(t_l(idx_l)), F_l(idx_l), 'Color', c_lugre, 'LineWidth', lw_base);
    h_s = plot(v_fun(t_s(idx_s)), F_s(idx_s), '--', 'Color', c_surr, 'LineWidth', lw_base);
end
title('Frictional Lag: F vs. v Hysteresis (\omega = 1, 10, 25 rad/s) [ode23tb]');
xlabel('Velocity v [m/s]'); ylabel('Friction Force F [N]');
xlim([0.4e-3, 2.6e-3]); ylim([1.0, 1.4]);
legend([h_l, h_s], {'LuGre (Truth)', 'Surrogate Model 15'}, 'Location', 'northeast');

exportgraphics(fig2, fullfile(save_dir, 'Fig2_FrictionalLag_Canonical.pdf'), 'Resolution', 300);
fprintf('   -> Saved: Fig2_FrictionalLag_Canonical.pdf\n');

%% ========================================================================
%% TEST 3: VARYING BREAKAWAY FORCE (ode23tb with Event Detection)
%% ========================================================================
fprintf('\n>>> Running Test 3: Varying Breakaway Force (ode23tb)...\n');

rate_list = [0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500];
m_unit = 1.0;

F_break_lugre = zeros(size(rate_list));
F_break_surr  = zeros(size(rate_list));

% Event function stops integration immediately when velocity hits 1e-4 m/s (breakaway)
event_opts = odeset(p.opts, 'Events', @breakaway_event);

for i = 1:length(rate_list)
    alpha = rate_list(i);
    t_max = max(2.0, (p.Fs * 1.5) / alpha);
    
    % Initial states: [v=0; z=0]
    Y0 = [0; 0];
    
    % Simulate dynamic acceleration: dv/dt = (alpha*t - F)/m
    [t_l, Y_l] = ode23tb(@(t, Y) odefun_breakaway(t, Y, alpha, m_unit, p, 'lugre'), [0, t_max], Y0, event_opts);
    [t_s, Y_s] = ode23tb(@(t, Y) odefun_breakaway(t, Y, alpha, m_unit, p, 'surrogate'), [0, t_max], Y0, event_opts);
    
    % Peak friction at breakaway moment
    F_l_all = compute_force(Y_l(:, 1), Y_l(:, 2), p, 'lugre');
    F_s_all = compute_force(Y_s(:, 1), Y_s(:, 2), p, 'surrogate');
    
    F_break_lugre(i) = F_l_all(end);
    F_break_surr(i)  = F_s_all(end);
end

fig3 = figure('Name', 'Test 3: Varying Breakaway Force', 'Color', 'w', 'Position', [200, 200, 700, 500]);
semilogx(rate_list, F_break_lugre, 'o-', 'Color', c_lugre, 'LineWidth', lw_base, 'MarkerSize', 6);
hold on; grid on;
semilogx(rate_list, F_break_surr, 's--', 'Color', c_surr, 'LineWidth', lw_base, 'MarkerSize', 6);
title('Varying Breakaway Force vs. Force Rate [ode23tb]');
xlabel('Force Rate dF_{ext}/dt [N/s]'); ylabel('Breakaway Force F_{break} [N]');
ylim([0.9, 1.6]);
legend({'LuGre (Truth)', 'Surrogate Model 15'}, 'Location', 'northeast');

exportgraphics(fig3, fullfile(save_dir, 'Fig3_VaryingBreakaway.pdf'), 'Resolution', 300);
fprintf('   -> Saved: Fig3_VaryingBreakaway.pdf\n');

%% ========================================================================
%% TEST 4: AUTONOMOUS STICK-SLIP (ode23tb Mass-Spring Plant)
%% ========================================================================
fprintf('\n>>> Running Test 4: Autonomous Stick-Slip Motion (ode23tb)...\n');

m_ss = 1.0;
k_ss = 2.0;
v_pull = 0.1; % Canonical LuGre Fig. 6
t_span = [0, 25];
Y0_ss = [0; 0; 0]; % [x=0; v=0; z=0]

[t_l, Y_l] = ode23tb(@(t, Y) odefun_stick_slip(t, Y, v_pull, m_ss, k_ss, p, 'lugre'), t_span, Y0_ss, p.opts);
[t_s, Y_s] = ode23tb(@(t, Y) odefun_stick_slip(t, Y, v_pull, m_ss, k_ss, p, 'surrogate'), t_span, Y0_ss, p.opts);

F_l = compute_force(Y_l(:, 2), Y_l(:, 3), p, 'lugre');
F_s = compute_force(Y_s(:, 2), Y_s(:, 3), p, 'surrogate');

fig4 = figure('Name', 'Test 4: Stick-Slip Comparison', 'Color', 'w', 'Position', [250, 250, 800, 600]);

subplot(3, 1, 1);
plot(t_l, Y_l(:, 1), 'Color', c_lugre, 'LineWidth', 1.2); hold on; grid on;
plot(t_s, Y_s(:, 1), '--', 'Color', c_surr, 'LineWidth', 1.2);
ylabel('Position x [m]'); title('Autonomous Stick-Slip Limit Cycle [ode23tb]');
legend({'LuGre (Truth)', 'Surrogate Model 15'}, 'Location', 'northwest');

subplot(3, 1, 2);
plot(t_l, Y_l(:, 2), 'Color', c_lugre, 'LineWidth', 1.2); hold on; grid on;
plot(t_s, Y_s(:, 2), '--', 'Color', c_surr, 'LineWidth', 1.2);
ylabel('Velocity v [m/s]'); ylim([-0.05, 0.5]);

subplot(3, 1, 3);
plot(t_l, F_l, 'Color', c_lugre, 'LineWidth', 1.2); hold on; grid on;
plot(t_s, F_s, '--', 'Color', c_surr, 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Friction F [N]'); ylim([0.5, 1.7]);

exportgraphics(fig4, fullfile(save_dir, 'Fig4_StickSlip_TimeHistory.pdf'), 'Resolution', 300);
fprintf('   -> Saved: Fig4_StickSlip_TimeHistory.pdf\n');

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
function [value, isterminal, direction] = breakaway_event(~, Y)
    % Triggers stop when velocity reaches 1e-4 m/s (macro-sliding onset)
    value = abs(Y(1)) - 1e-4;
    isterminal = 1; % Stop integration
    direction = 1;  % Positive crossing
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