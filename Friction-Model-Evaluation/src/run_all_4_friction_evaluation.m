%% ========================================================================
%   MASTER FRICTION EVALUATION SUITE: LuGre (Truth) vs. Surrogate Model 2B
%   Author: Shu Kikuchi
%   Coverage: 
%     Test 1: Pre-Sliding Hysteresis (F vs. x) [Sec. IV-A]
%     Test 2: Frictional Lag (F vs. v)         [Sec. IV-B / Fig. 3]
%     Test 3: Varying Breakaway Force (F vs dF/dt) [Sec. IV-C / Fig. 4]
%     Test 4: Autonomous Stick-Slip (Mass-Spring)  [Sec. IV-D / Fig. 5-6]
% ========================================================================
clearvars; clc; close all;

%% --- 0. GLOBAL SETUP & MODEL DEFINITIONS ---
fprintf('====================================================================\n');
fprintf('       RUNNING COMPLETE 4-PHENOMENON EVALUATION SUITE              \n');
fprintf('====================================================================\n');

% Output Directory Setup
save_dir = fullfile(pwd, '..', 'tmp-outputs', 'model15');
if ~exist(save_dir, 'dir'), mkdir(save_dir); end

% --- Physical LuGre Parameters (Ground Truth) ---
p.sigma0 = 1e5;
p.sigma1 = sqrt(p.sigma0);
p.sigma2 = 0.4;
p.Fc     = 1.0;
p.Fs     = 1.5;
p.vs     = 0.001;

% --- Surrogate Model 2B Constants (PySR Discovered) ---
p.S0 = 1e5;
p.C1 = 0.85228235;
p.C2 = -0.13998671;
p.C3 = 6.7612834;
p.C4 = 7.2667336;

% Simulation Time-Step
dt = 1e-4; % 0.1 ms fixed grid

% Visual Styles (Strict Academic Standard)
c_lugre = [0, 0.447, 0.741];      % Blue Solid: Ground Truth
c_surr  = [0.850, 0.325, 0.098];    % Red Dashed: Surrogate Model
lw_base = 1.5;

%% ========================================================================
%% TEST 1: PRE-SLIDING HYSTERESIS (F vs. x)
%% ========================================================================
fprintf('\n>>> Running Test 1: Pre-Sliding Hysteresis (F vs. x)...\n');

% 1. Canonical Paper Plot (A = 4 um, w = 1, 10, 25 rad/s)
w_canon = [1, 10, 25];
A_canon = 4e-6; % 4 um (Well below delta = 15 um)

fig1 = figure('Name', 'Test 1: Pre-Sliding Canonical', 'Color', 'w', 'Position', [100, 100, 700, 500]);
hold on; grid on;

for w_val = w_canon
    t_end = max(10, (2*pi/w_val)*4);
    t = 0:dt:t_end;
    v_input = A_canon * w_val * cos(w_val * t);
    x_input = A_canon * sin(w_val * t);
    
    % Simulate both models
    [~, F_lugre] = sim_kinematic_lugre(v_input, dt, p);
    [~, F_surr]  = sim_kinematic_surrogate(v_input, dt, p);
    
    % Steady-state cycle (last cycle)
    idx_cycle = t > (t_end - 2*pi/w_val);
    plot(x_input(idx_cycle), F_lugre(idx_cycle), 'Color', c_lugre, 'LineWidth', lw_base);
    plot(x_input(idx_cycle), F_surr(idx_cycle), '--', 'Color', c_surr, 'LineWidth', lw_base);
end
title('Pre-Sliding Hysteresis: F vs. x (A = 4 \mu m)');
xlabel('Displacement x [m]'); ylabel('Friction Force F [N]');
xlim([-5e-6, 5e-6]); ylim([-0.5, 0.5]);
legend({'LuGre (Truth)', 'Surrogate Model 2B'}, 'Location', 'northwest');

exportgraphics(fig1, fullfile(save_dir, 'Fig1_Presliding_Canonical.pdf'), 'Resolution', 300);
fprintf('   -> Saved: Fig1_Presliding_Canonical.pdf\n');

%% ========================================================================
%% TEST 2: FRICTIONAL LAG (F vs. v)
%% ========================================================================
fprintf('\n>>> Running Test 2: Frictional Lag (Hess & Soom / LuGre Fig. 3)...\n');

% Canonical Setup: v = 1.5 + 1.0*sin(w*t) mm/s
lag_w_list = [1, 10, 25];
v_bias = 1.5e-3;
v_amp  = 1.0e-3;

fig2 = figure('Name', 'Test 2: Frictional Lag Canonical', 'Color', 'w', 'Position', [150, 150, 750, 500]);
hold on; grid on;

for w_val = lag_w_list
    t_end = max(15, (2*pi/w_val)*4);
    t = 0:dt:t_end;
    v_input = v_bias + v_amp * sin(w_val * t);
    
    [~, F_lugre] = sim_kinematic_lugre(v_input, dt, p);
    [~, F_surr]  = sim_kinematic_surrogate(v_input, dt, p);
    
    idx_cycle = t > (t_end - 2*pi/w_val);
    h_l = plot(v_input(idx_cycle), F_lugre(idx_cycle), 'Color', c_lugre, 'LineWidth', lw_base);
    h_s = plot(v_input(idx_cycle), F_surr(idx_cycle), '--', 'Color', c_surr, 'LineWidth', lw_base);
end
title('Frictional Lag: F vs. v Hysteresis (\omega = 1, 10, 25 rad/s)');
xlabel('Velocity v [m/s]'); ylabel('Friction Force F [N]');
xlim([0.4e-3, 2.6e-3]); ylim([1.0, 1.4]);
legend([h_l, h_s], {'LuGre (Truth)', 'Surrogate Model 2B'}, 'Location', 'northeast');

exportgraphics(fig2, fullfile(save_dir, 'Fig2_FrictionalLag_Canonical.pdf'), 'Resolution', 300);
fprintf('   -> Saved: Fig2_FrictionalLag_Canonical.pdf\n');

%% ========================================================================
%% TEST 3: VARYING BREAKAWAY FORCE (F_break vs. dF/dt)
%% ========================================================================
fprintf('\n>>> Running Test 3: Varying Breakaway Force (LuGre Fig. 4)...\n');

% Force Rates: 0.1 N/s up to 500 N/s (4 decades)
rate_list = [0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500];
m_unit = 1.0; % 1 kg

F_break_lugre = zeros(size(rate_list));
F_break_surr  = zeros(size(rate_list));

for i = 1:length(rate_list)
    alpha = rate_list(i);
    t_max = max(2.0, (p.Fs * 1.5) / alpha);
    t = 0:dt:t_max;
    
    F_ext = alpha * t;
    
    % Simulate dynamic acceleration: m*dv/dt = F_ext - F_friction
    [~, v_l, F_l] = sim_dynamic_mass(F_ext, m_unit, dt, p, 'lugre');
    [~, v_s, F_s] = sim_dynamic_mass(F_ext, m_unit, dt, p, 'surrogate');
    
    % Breakaway detected when velocity exceeds micro-motion threshold (1e-4 m/s)
    idx_l = find(abs(v_l) > 1e-4, 1, 'first');
    idx_s = find(abs(v_s) > 1e-4, 1, 'first');
    
    if isempty(idx_l), F_break_lugre(i) = max(F_l); else, F_break_lugre(i) = F_l(idx_l); end
    if isempty(idx_s), F_break_surr(i)  = max(F_s); else, F_break_surr(i)  = F_s(idx_s); end
end

fig3 = figure('Name', 'Test 3: Varying Breakaway Force', 'Color', 'w', 'Position', [200, 200, 700, 500]);
semilogx(rate_list, F_break_lugre, 'o-', 'Color', c_lugre, 'LineWidth', lw_base, 'MarkerSize', 6);
hold on; grid on;
semilogx(rate_list, F_break_surr, 's--', 'Color', c_surr, 'LineWidth', lw_base, 'MarkerSize', 6);
title('Varying Breakaway Force vs. Force Rate (m = 1 kg)');
xlabel('Force Rate dF_{ext}/dt [N/s]'); ylabel('Breakaway Force F_{break} [N]');
ylim([0.9, 1.6]);
legend({'LuGre (Truth)', 'Surrogate Model 2B'}, 'Location', 'northeast');

exportgraphics(fig3, fullfile(save_dir, 'Fig3_VaryingBreakaway.pdf'), 'Resolution', 300);
fprintf('   -> Saved: Fig3_VaryingBreakaway.pdf\n');

%% ========================================================================
%% TEST 4: AUTONOMOUS STICK-SLIP (Mass-Spring System, LuGre Fig. 5-6)
%% ========================================================================
fprintf('\n>>> Running Test 4: Autonomous Stick-Slip Motion...\n');

% Setup canonical LuGre parameters: m = 1 kg, k = 2 N/m, v_pull = 0.1 m/s
m_ss = 1.0;
k_ss = 2.0;
v_pull = 0.1;
t_ss = 0:dt:25; % 25 seconds

[x_l, v_l, F_l] = sim_stick_slip(v_pull, m_ss, k_ss, t_ss, dt, p, 'lugre');
[x_s, v_s, F_s] = sim_stick_slip(v_pull, m_ss, k_ss, t_ss, dt, p, 'surrogate');

fig4 = figure('Name', 'Test 4: Stick-Slip Comparison', 'Color', 'w', 'Position', [250, 250, 800, 600]);

subplot(3, 1, 1);
plot(t_ss, x_l, 'Color', c_lugre, 'LineWidth', 1.2); hold on; grid on;
plot(t_ss, x_s, '--', 'Color', c_surr, 'LineWidth', 1.2);
ylabel('Position x [m]'); title('Autonomous Stick-Slip Limit Cycle (m = 1 kg, k = 2 N/m, v_{pull} = 0.1 m/s)');
legend({'LuGre (Truth)', 'Surrogate Model 2B'}, 'Location', 'northwest');

subplot(3, 1, 2);
plot(t_ss, v_l, 'Color', c_lugre, 'LineWidth', 1.2); hold on; grid on;
plot(t_ss, v_s, '--', 'Color', c_surr, 'LineWidth', 1.2);
ylabel('Velocity v [m/s]'); ylim([-0.05, 0.5]);

subplot(3, 1, 3);
plot(t_ss, F_l, 'Color', c_lugre, 'LineWidth', 1.2); hold on; grid on;
plot(t_ss, F_s, '--', 'Color', c_surr, 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Friction F [N]'); ylim([0.5, 1.7]);

exportgraphics(fig4, fullfile(save_dir, 'Fig4_StickSlip_TimeHistory.pdf'), 'Resolution', 300);
fprintf('   -> Saved: Fig4_StickSlip_TimeHistory.pdf\n');

fprintf('\n====================================================================\n');
fprintf('   ALL 4 BENCHMARKS SUCCESSFULLY EXECUTED AND EXPORTED TO PDF!      \n');
fprintf('   Folder: %s\n', save_dir);
fprintf('====================================================================\n');


%% ========================================================================
%% HELPER FUNCTIONS: CLOSED-FORM NUMERICAL SIMULATION ENGINES
%% ========================================================================

% --- 1. Pure Kinematic LuGre Engine ---
function [z_arr, F_arr] = sim_kinematic_lugre(v_vec, dt, p)
    N = length(v_vec);
    z = 0;
    z_arr = zeros(1, N);
    F_arr = zeros(1, N);
    
    for i = 1:N
        v = v_vec(i);
        gv = p.Fc + (p.Fs - p.Fc) * exp(-(v / p.vs)^2);
        dz = v - (p.sigma0 * abs(v) / gv) * z;
        z = z + dt * dz;
        
        z_arr(i) = z;
        F_arr(i) = p.sigma0 * z + p.sigma1 * dz + p.sigma2 * v;
    end
end

% --- 2. Pure Kinematic Surrogate Model 2B Engine ---
function [z_arr, F_arr] = sim_kinematic_surrogate(v_vec, dt, p)
    N = length(v_vec);
    z = 0;
    z_arr = zeros(1, N);
    F_arr = zeros(1, N);
    
    for i = 1:N
        v = v_vec(i);
        % Model 2B Non-Linear Formulation:
        term1 = abs(p.C1 * p.S0 * v);
        term2 = abs(p.C3 + p.C2 * (p.S0^2 * v * z));
        dz = v - z * (term1 + term2 - p.C4);
        z = z + dt * dz;
        
        z_arr(i) = z;
        F_arr(i) = p.sigma0 * z + p.sigma1 * dz + p.sigma2 * v;
    end
end

% --- 3. Dynamic Mass Under External Force (Test 3) ---
function [x_arr, v_arr, F_arr] = sim_dynamic_mass(F_ext_vec, m, dt, p, model_type)
    N = length(F_ext_vec);
    x = 0; v = 0; z = 0;
    x_arr = zeros(1, N); v_arr = zeros(1, N); F_arr = zeros(1, N);
    
    for i = 1:N
        F_ext = F_ext_vec(i);
        
        if strcmp(model_type, 'lugre')
            gv = p.Fc + (p.Fs - p.Fc) * exp(-(v / p.vs)^2);
            dz = v - (p.sigma0 * abs(v) / gv) * z;
        else
            term1 = abs(p.C1 * p.S0 * v);
            term2 = abs(p.C3 + p.C2 * (p.S0^2 * v * z));
            dz = v - z * (term1 + term2 - p.C4);
        end
        
        F_fric = p.sigma0 * z + p.sigma1 * dz + p.sigma2 * v;
        
        % State updates
        dv = (F_ext - F_fric) / m;
        v = v + dt * dv;
        x = x + dt * v;
        z = z + dt * dz;
        
        x_arr(i) = x; v_arr(i) = v; F_arr(i) = F_fric;
    end
end

% --- 4. Dynamic Mass-Spring Stick-Slip Engine (Test 4) ---
function [x_arr, v_arr, F_arr] = sim_stick_slip(v_pull, m, k, t_vec, dt, p, model_type)
    N = length(t_vec);
    x = 0; v = 0; z = 0;
    x_arr = zeros(1, N); v_arr = zeros(1, N); F_arr = zeros(1, N);
    
    for i = 1:N
        t = t_vec(i);
        y_pull = v_pull * t;
        F_spring = k * (y_pull - x);
        
        if strcmp(model_type, 'lugre')
            gv = p.Fc + (p.Fs - p.Fc) * exp(-(v / p.vs)^2);
            dz = v - (p.sigma0 * abs(v) / gv) * z;
        else
            term1 = abs(p.C1 * p.S0 * v);
            term2 = abs(p.C3 + p.C2 * (p.S0^2 * v * z));
            dz = v - z * (term1 + term2 - p.C4);
        end
        
        F_fric = p.sigma0 * z + p.sigma1 * dz + p.sigma2 * v;
        
        dv = (F_spring - F_fric) / m;
        v = v + dt * dv;
        x = x + dt * v;
        z = z + dt * dz;
        
        x_arr(i) = x; v_arr(i) = v; F_arr(i) = F_fric;
    end
end