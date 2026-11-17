%% ========================================================================
%   MASTER 1-DOF ACTUATOR BENCHMARK SUITE: ARMSTRONG'S 4 CANONICAL REGIMES
%   Control Architecture: 2-DOF Control (PD Feedback + Dynamics Feedforward)
%   Plant: Rotor Inertia (J) + True LuGre Friction Dynamics (100 kHz ETD)
%   Taxonomy: Brian Armstrong-Helouvry et al. (1994, Automatica)
%     - Regime I   : Regulator & Precision Micro-Pointing (v = 0, Stiction)
%     - Regime II  : Tracking with Velocity Reversals (v crosses 0)
%     - Regime III : Tracking at Low Velocities (Stribeck Negative Damping)
%     - Regime IV  : Tracking at High Velocities (Viscous Drag Dominance)
%   Comparison:
%     - Case A: Baseline PD (No compensation)
%     - Case B: PD + Classical Coulomb + Viscous Feedforward
%     - Case C: PD + LuGre Friction Observer (Canudas de Wit 1995)
%     - Case D: PD + Proposed Symbolic Model 15 (Algebraic Backward Euler)
% ========================================================================
clearvars; clc; close all;

fprintf('====================================================================\n');
fprintf('   STARTING 1-DOF BENCHMARK SUITE: ARMSTRONG 4 CANONICAL REGIMES    \n');
fprintf('====================================================================\n');

%% --- 1. SYSTEM PARAMETERS ---
% =========================================================================
% --- NOISE & SENSOR DISTURBANCE SETTINGS (TOGGLE SWITCH) ---
% =========================================================================
p.use_noise = false;    % <<< SWITCH: Set to false (Phase 1) or true (Phase 2)

% Hardware Noise Parameters (Standard 10:1 Robot Joint Actuator)
p.enc_bits    = 16;     % 16-bit encoder (65,536 counts/rev)
p.fc_vel      = 30.0;   % Velocity Low-Pass Filter cutoff frequency [Hz]
p.tau_cog_amp = 0.03;   % Cogging torque amplitude [Nm] (~3% rated torque)
p.tau_adc_std = 0.01;   % Current sensor ADC noise standard deviation [Nm]

% =========================================================================
% --- OUTPUT DIRECTORY & AUTOMATIC MODE TAGGING ---
% =========================================================================
date_and_exp = '26-11-14_armstrong-4regimes-suite';

if p.use_noise
    mode_tag = 'Phase2_Noisy_Robustness';
else
    mode_tag = 'Phase1_Ideal_ZeroNoise';
end

save_dir = fullfile(pwd, 'tmp-outputs', 'model15', '1-DOF', date_and_exp, mode_tag);
if ~exist(save_dir, 'dir'), mkdir(save_dir); end
fprintf('   -> Saving outputs to: %s\n', save_dir);

% Actuator Inertia & Time Base
p.J  = 0.01;            % Motor + Load Inertia [kg*m^2]
f_ctrl = 1000;          % Control frequency: 1 kHz
Ts = 1 / f_ctrl;        % Control period: 1 ms

% Feedback Controller Gains (PD) - Satisfies SPR condition: Kv > (sigma0/sigma1)*J = 3.162
Kp_stiff = 200.0;       % High-stiffness tracking gain [Nm/rad] (fn ≈ 22.5 Hz)
Kp_soft  = 50.0;        % Compliant contact gain [Nm/rad] (fn ≈ 11.2 Hz)
Kv       = 4.0;         % Velocity damping gain [Nm*s/rad] (SPR-stable!)

% True Actuator Friction Parameters (LuGre Plant)
p.sigma0 = 1e5;
p.sigma1 = sqrt(p.sigma0);
p.sigma2 = 0.4;         % Linear viscous coefficient (b)
p.Fc     = 1.0;         % Coulomb level [Nm]
p.Fs     = 1.5;         % Static level [Nm]
p.vs     = 0.001;       % Stribeck velocity [rad/s]

% Proposed Surrogate Model 15 Constants
p.S0 = 1e5;
p.C1 = 0.00042237472;
p.C2 = 2.807613;
p.K  = (p.S0^2) * p.C1;

methods = {'No Comp (PD)', 'Coulomb+Viscous', 'LuGre (Truth)', 'Model 15 (Proposed)'};
c_map = {
    [0.2, 0.2, 0.2], ...        % Gray: No Comp
    [0.929, 0.694, 0.125], ...  % Yellow-Orange: Coulomb+Viscous
    [0.0, 0.447, 0.741], ...    % Blue: LuGre
    [0.850, 0.325, 0.098]       % Red-Orange: Model 15
};

%% ========================================================================
%% --- SECTION 2: EXECUTION TIME BENCHMARK (Calculation Cost in us)
%% ========================================================================
fprintf('\n>>> Benchmarking Model Execution Time (100,000 evaluations)...\n');
v_test = 0.05; z_test = 1e-5; e_test = 0.001; k_obs = 0.5;
N_eval = 100000;

% 1. Coulomb + Viscous
tic;
for i = 1:N_eval
    T_cv = p.Fc * sign(v_test) + p.sigma2 * v_test;
end
t_cv = (toc / N_eval) * 1e6;

% 2. Full LuGre Update with Observer (Exponential form)
tic;
z_l = z_test;
for i = 1:N_eval
    gv = p.Fc + (p.Fs - p.Fc) * exp(-(v_test / p.vs)^2);
    beta_l = (p.sigma0 * abs(v_test)) / gv;
    z_ss_l = (v_test + k_obs * e_test) / beta_l;
    z_l = z_ss_l + (z_l - z_ss_l) * exp(-beta_l * Ts);
    dz_l = v_test - beta_l * z_l + k_obs * e_test;
    T_lugre = p.sigma0 * z_l + p.sigma1 * dz_l + p.sigma2 * v_test;
end
t_lugre = (toc / N_eval) * 1e6;

% 3. Model 15 Update with Observer (Pure Algebraic Backward Euler - ZERO exp!)
tic;
z_s = z_test;
for i = 1:N_eval
    denom_s = p.K * (v_test^2) + p.C2;
    beta_s  = abs(v_test) * p.S0 * (1 - 1 / denom_s);
    z_s     = (z_s + Ts * (v_test + k_obs * e_test)) / (1 + beta_s * Ts);
    dz_s    = v_test - beta_s * z_s + k_obs * e_test;
    T_s15   = p.sigma0 * z_s + p.sigma1 * dz_s + p.sigma2 * v_test;
end
t_s15 = (toc / N_eval) * 1e6;

t_exec = [0.05, t_cv, t_lugre, t_s15];
fprintf('   -> Coulomb+Viscous : %.3f us\n', t_cv);
fprintf('   -> LuGre Observer  : %.3f us\n', t_lugre);
fprintf('   -> Model 15 (Alg.) : %.3f us (%.1f%% faster than LuGre)\n', t_s15, (1 - t_s15/t_lugre)*100);

%% ========================================================================
%% REGIME I: REGULATOR & PRECISION MICRO-POINTING (Stiction & Anti-Hunting)
%% ========================================================================
fprintf('\n>>> Simulating Regime I: Regulator / Precision Micro-Pointing...\n');
t_end_1 = 12.0;
t_vec1  = 0:Ts:t_end_1;
N1 = length(t_vec1);

% Micro-step sequence [start_time, target_from, target_to]
steps1 = [
    2.0,  0.000,  0.005;   % Step 1: 0 -> 5 mrad
    5.0,  0.005,  0.015;   % Step 2: 5 -> 15 mrad
    8.0,  0.015,  0.010    % Step 3: Reversal to 10 mrad
];
T_trans1 = 0.4; % 400 ms smooth S-curve transition

th_d1 = zeros(size(t_vec1));
om_d1 = zeros(size(t_vec1));
al_d1 = zeros(size(t_vec1));

for s_idx = 1:size(steps1, 1)
    t_s      = steps1(s_idx, 1);
    th_from  = steps1(s_idx, 2);
    th_to    = steps1(s_idx, 3);
    delta_th = th_to - th_from;
    
    idx_trans = find(t_vec1 >= t_s & t_vec1 < (t_s + T_trans1));
    tau = (t_vec1(idx_trans) - t_s) / T_trans1;
    
    th_d1(idx_trans) = th_from + delta_th * 0.5 * (1 - cos(pi * tau));
    om_d1(idx_trans) = delta_th * (pi / (2 * T_trans1)) * sin(pi * tau);
    al_d1(idx_trans) = delta_th * ((pi^2) / (2 * T_trans1^2)) * cos(pi * tau);
    
    if s_idx < size(steps1, 1)
        t_next = steps1(s_idx + 1, 1);
    else
        t_next = t_end_1;
    end
    idx_hold = find(t_vec1 >= (t_s + T_trans1) & t_vec1 <= t_next);
    th_d1(idx_hold) = th_to;
end

results_r1 = cell(4, 1);
for m_idx = 1:4
    results_r1{m_idx} = run_actuator_sim(t_vec1, th_d1, om_d1, al_d1, methods{m_idx}, p, Kp_soft, Kv, Ts);
end

% --- Plot Fig 1: Regime I Subplots ---
fig1 = figure('Name', 'Regime I: Regulator (Micro-Pointing)', 'Color', 'w', 'Position', [50, 50, 950, 750]);

subplot(3, 1, 1); hold on; grid on;
plot(t_vec1, th_d1, 'k--', 'LineWidth', 1.2);
for m_idx = 1:4
    plot(t_vec1, results_r1{m_idx}.th, 'Color', c_map{m_idx}, 'LineWidth', 1.2);
end
ylabel('\theta [rad]'); title('Regime I (Regulator): Micro-Step Pointing under Compliance (K_p = 50)');
legend([{'Target \theta_d'}, methods], 'Location', 'southeast');

subplot(3, 1, 2); hold on; grid on;
for m_idx = 1:4
    plot(t_vec1, results_r1{m_idx}.e, 'Color', c_map{m_idx}, 'LineWidth', 1.2);
end
ylabel('Error e(t) [rad]'); title('Positioning Error (Steady-State Stiction & Hunting Suppression)');

subplot(3, 1, 3); hold on; grid on;
for m_idx = 1:4
    plot(t_vec1, results_r1{m_idx}.T_cmd, 'Color', c_map{m_idx}, 'LineWidth', 1.1);
end
xlabel('Time [s]'); ylabel('Torque T^* [Nm]'); title('Commanded Torque T^*(t) (Zero Standstill Quivering)');

fig1_path = fullfile(save_dir, sprintf('Fig1_RegimeI_Regulator_%s.pdf', mode_tag));
exportgraphics(fig1, fig1_path, 'ContentType', 'vector');
fprintf('   -> Saved Figure 1: %s\n', fig1_path);

%% ========================================================================
%% REGIME II: TRACKING WITH VELOCITY REVERSALS (Zero-Crossing Deadband)
%% ========================================================================
fprintf('\n>>> Simulating Regime II: Velocity Reversals (Zero-Crossing)...\n');
t_end_2 = 35.0; % Complete cycle at 0.2 rad/s (Period = 31.4 s)
t_vec2  = 0:Ts:t_end_2;
N2 = length(t_vec2);

% Low-speed deliberate zero-crossing: v_max = 0.02 rad/s
A_rev = 0.1; w_rev = 0.2;
th_d2 = A_rev * sin(w_rev * t_vec2);
om_d2 = A_rev * w_rev * cos(w_rev * t_vec2);
al_d2 = -A_rev * (w_rev^2) * sin(w_rev * t_vec2);

results_r2 = cell(4, 1);
for m_idx = 1:4
    results_r2{m_idx} = run_actuator_sim(t_vec2, th_d2, om_d2, al_d2, methods{m_idx}, p, Kp_stiff, Kv, Ts);
end

% --- Plot Fig 2: Regime II Subplots ---
fig2 = figure('Name', 'Regime II: Velocity Reversals', 'Color', 'w', 'Position', [80, 70, 950, 750]);

subplot(3, 1, 1); hold on; grid on;
plot(t_vec2, th_d2, 'k--', 'LineWidth', 1.2);
for m_idx = 1:4
    plot(t_vec2, results_r2{m_idx}.th, 'Color', c_map{m_idx}, 'LineWidth', 1.2);
end
ylabel('\theta [rad]'); title('Regime II (Velocity Reversals): Zero-Crossing Tracking (v_{max} = 20 mm/s)');
legend([{'Desired \theta_d'}, methods], 'Location', 'northeast');

subplot(3, 1, 2); hold on; grid on;
for m_idx = 1:4
    plot(t_vec2, results_r2{m_idx}.e, 'Color', c_map{m_idx}, 'LineWidth', 1.2);
end
ylabel('Error e(t) [rad]'); title('Tracking Error (Lost-Motion Deadband Elimination)');

subplot(3, 1, 3); hold on; grid on;
for m_idx = 1:4
    plot(t_vec2, results_r2{m_idx}.T_cmd, 'Color', c_map{m_idx}, 'LineWidth', 1.1);
end
xlabel('Time [s]'); ylabel('Torque T^* [Nm]'); title('Commanded Torque T^*(t) (Bumpless Pre-Sliding Transition)');

fig2_path = fullfile(save_dir, sprintf('Fig2_RegimeII_Reversal_%s.pdf', mode_tag));
exportgraphics(fig2, fig2_path, 'ContentType', 'vector');
fprintf('   -> Saved Figure 2: %s\n', fig2_path);

%% ========================================================================
%% REGIME III: TRACKING AT LOW VELOCITIES (Compliant Stribeck Crawl)
%% ========================================================================
fprintf('\n>>> Simulating Regime III: Low-Velocity Compliant Stribeck Crawl...\n');
t_end_3 = 20.0;
t_vec3  = 0:Ts:t_end_3;

% Ultra-slow crawl directly on the Stribeck cliff (v_max = 1.2 mm/s)
A_crawl = 0.003; w_crawl = 0.4;
th_d3 = A_crawl * (1 - cos(w_crawl * t_vec3));
om_d3 = A_crawl * w_crawl * sin(w_crawl * t_vec3);
al_d3 = A_crawl * (w_crawl^2) * cos(w_crawl * t_vec3);

results_r3 = cell(4, 1);
for m_idx = 1:4
    results_r3{m_idx} = run_actuator_sim(t_vec3, th_d3, om_d3, al_d3, methods{m_idx}, p, Kp_soft, Kv, Ts);
end

% --- Plot Fig 3: Regime III Subplots ---
fig3 = figure('Name', 'Regime III: Compliant Stribeck Crawl', 'Color', 'w', 'Position', [110, 90, 950, 750]);

subplot(3, 1, 1); hold on; grid on;
plot(t_vec3, th_d3, 'k--', 'LineWidth', 1.2);
for m_idx = 1:4
    plot(t_vec3, results_r3{m_idx}.th, 'Color', c_map{m_idx}, 'LineWidth', 1.2);
end
ylabel('\theta [rad]'); title('Regime III (Low-Velocity Stribeck): Compliant Crawl (K_p = 50, v_{max} = 1.2 mm/s)');
legend([{'Desired \theta_d'}, methods], 'Location', 'northeast');

subplot(3, 1, 2); hold on; grid on;
for m_idx = 1:4
    plot(t_vec3, results_r3{m_idx}.e, 'Color', c_map{m_idx}, 'LineWidth', 1.2);
end
ylabel('Error e(t) [rad]'); title('Tracking Error (Stick-Slip Limit Cycle Elimination)');

subplot(3, 1, 3); hold on; grid on;
for m_idx = 1:4
    plot(t_vec3, results_r3{m_idx}.T_cmd, 'Color', c_map{m_idx}, 'LineWidth', 1.1);
end
xlabel('Time [s]'); ylabel('Torque T^* [Nm]'); title('Commanded Torque T^*(t) (Negative Damping Compensation)');

fig3_path = fullfile(save_dir, sprintf('Fig3_RegimeIII_StribeckCrawl_%s.pdf', mode_tag));
exportgraphics(fig3, fig3_path, 'ContentType', 'vector');
fprintf('   -> Saved Figure 3: %s\n', fig3_path);

%% ========================================================================
%% REGIME IV: TRACKING AT HIGH VELOCITIES (Viscous Drag Dominance)
%% ========================================================================
fprintf('\n>>> Simulating Regime IV: High-Velocity Dynamic Tracking...\n');
t_end_4 = 10.0;
t_vec4  = 0:Ts:t_end_4;

% High-speed sprint: v_max = 5.0 rad/s (approx 48 RPM, viscous drag = 2.0 Nm!)
A_fast = 2.0; w_fast = 2.5;
th_d4 = A_fast * sin(w_fast * t_vec4);
om_d4 = A_fast * w_fast * cos(w_fast * t_vec4);
al_d4 = -A_fast * (w_fast^2) * sin(w_fast * t_vec4);

results_r4 = cell(4, 1);
for m_idx = 1:4
    results_r4{m_idx} = run_actuator_sim(t_vec4, th_d4, om_d4, al_d4, methods{m_idx}, p, Kp_stiff, Kv, Ts);
end

% --- Plot Fig 4: Regime IV Subplots ---
fig4 = figure('Name', 'Regime IV: High-Velocity Dynamic Tracking', 'Color', 'w', 'Position', [140, 110, 950, 750]);

subplot(3, 1, 1); hold on; grid on;
plot(t_vec4, th_d4, 'k--', 'LineWidth', 1.2);
for m_idx = 1:4
    plot(t_vec4, results_r4{m_idx}.th, 'Color', c_map{m_idx}, 'LineWidth', 1.2);
end
ylabel('\theta [rad]'); title('Regime IV (High-Velocity): Viscous Drag Dominance (v_{max} = 5.0 rad/s)');
legend([{'Desired \theta_d'}, methods], 'Location', 'northeast');

subplot(3, 1, 2); hold on; grid on;
for m_idx = 1:4
    plot(t_vec4, results_r4{m_idx}.e, 'Color', c_map{m_idx}, 'LineWidth', 1.2);
end
ylabel('Error e(t) [rad]'); title('Tracking Error (Viscous Damping Compensation)');

subplot(3, 1, 3); hold on; grid on;
for m_idx = 1:4
    plot(t_vec4, results_r4{m_idx}.T_cmd, 'Color', c_map{m_idx}, 'LineWidth', 1.1);
end
xlabel('Time [s]'); ylabel('Torque T^* [Nm]'); title('Commanded Torque T^*(t) (Seamless Multi-Regime Scaling)');

fig4_path = fullfile(save_dir, sprintf('Fig4_RegimeIV_HighSpeed_%s.pdf', mode_tag));
exportgraphics(fig4, fig4_path, 'ContentType', 'vector');
fprintf('   -> Saved Figure 4: %s\n', fig4_path);

%% ========================================================================
%% --- SECTION 3: METRICS COMPUTATION & THESIS TABLE EXPORT ---
%% ========================================================================
rms_err  = zeros(4, 1);
max_err  = zeros(4, 1);
delay_ms = zeros(4, 1);
rms_Tfb  = zeros(4, 1);

% Identify reference zero-crossing points in Regime II (Velocity Reversals)
zc_indices = find(diff(sign(om_d2)) ~= 0);

for m_idx = 1:4
    % Tracking error in Regime II (Reversal benchmark)
    e2 = results_r2{m_idx}.e;
    rms_err(m_idx) = sqrt(mean(e2.^2));
    max_err(m_idx) = max(abs(e2));
    
    % Feedback Effort RMS in Regime II: T_fb -> 0 (Armstrong 1991/1994 Metric!)
    T_fb_vec = results_r2{m_idx}.T_fb;
    rms_Tfb(m_idx) = sqrt(mean(T_fb_vec.^2));
    
    % Zero-crossing deadband delay (in ms) during Regime II reversals
    delays = [];
    om_act = results_r2{m_idx}.om;
    for zc = zc_indices(2:end-1)
        target_sign = sign(om_d2(zc + 1));
        post_window = zc:min(N2, zc + round(2.0/Ts));
        idx_cross = find(sign(om_act(post_window)) == target_sign, 1, 'first');
        if ~isempty(idx_cross)
            delays = [delays, (idx_cross - 1) * Ts * 1000];
        end
    end
    if isempty(delays)
        delay_ms(m_idx) = 2000.0; % Stalled across window
    else
        delay_ms(m_idx) = mean(delays);
    end
end

% Construct Comprehensive Thesis Summary Table
T_results = table(methods', rms_err, max_err, delay_ms, rms_Tfb, t_exec', ...
    'VariableNames', {'Compensation_Method', 'RMS_Error_rad', 'Max_Error_rad', 'ZeroCross_Delay_ms', 'Feedback_Effort_RMS_Nm', 'Execution_Time_us'});

fprintf('\n====================================================================================================\n');
fprintf('                           FINAL THESIS EVALUATION TABLE (ARMSTRONG 1994 SUITE)                    \n');
fprintf('====================================================================================================\n');
disp(T_results);
fprintf('====================================================================================================\n');

% Export Table as CSV (Excel) and formatted TXT
table_csv_path = fullfile(save_dir, sprintf('Table_Results_%s.csv', mode_tag));
writetable(T_results, table_csv_path);

table_txt_path = fullfile(save_dir, sprintf('Table_Results_%s.txt', mode_tag));
writetable(T_results, table_txt_path, 'Delimiter', '\t');
fprintf('   -> Saved Table CSV & TXT to: %s\n', save_dir);

%% ========================================================================
%% --- SECTION 4: HELPER FUNCTION (ACTUATOR SIMULATION & CONTROL LOOP) ---
%% ========================================================================

function res = run_actuator_sim(t_vec, th_d, om_d, al_d, method, p, Kp, Kv, Ts)
    N = length(t_vec);
    th_arr   = zeros(1, N);
    om_arr   = zeros(1, N);
    T_cmd    = zeros(1, N);
    T_fb_arr = zeros(1, N);
    
    % Initial states
    th = th_d(1);
    om = om_d(1);
    z_true = 0.0;    % Physical plant LuGre state
    z_ctrl = 0.0;    % Controller feedforward internal state

    % Sensor memory and filter constants
    th_meas_prev = th;
    om_meas      = om;
    d_th_res     = (2 * pi) / (2^p.enc_bits);
    alpha_lpf    = (2 * pi * p.fc_vel * Ts) / (1 + 2 * pi * p.fc_vel * Ts);
    
    % Sub-stepping parameters (100 substeps per control step -> 100 kHz continuous plant)
    n_sub = 100;
    dt = Ts / n_sub;
    
    for k = 1:N
        % =================================================================
        % 0. Sensor Measurement Stage (Quantization & Filter Lag)
        % =================================================================
        if p.use_noise
            % 16-bit Quantization + Gaussian Jitter
            th_quant = d_th_res * round(th / d_th_res);
            th_meas  = th_quant + (0.2 * d_th_res) * randn();
            
            % Numerical velocity differentiation + 1st-order LPF
            om_raw       = (th_meas - th_meas_prev) / Ts;
            th_meas_prev = th_meas;
            om_meas      = (1 - alpha_lpf) * om_meas + alpha_lpf * om_raw;
        else
            % Ideal noiseless read
            th_meas = th;
            om_meas = om;
        end
        
        % 1. Feedback Torque (PD) - Uses measured states
        T_fb = Kp * (th_d(k) - th_meas) + Kv * (om_d(k) - om_meas);
        
        % 2. Inertial Feedforward (Acceleration only; linear viscous damping omitted)
        T_ff = p.J * al_d(k);
        
        % 3. Friction Feedforward (Unified Model with Observer & Anti-Hunting Deadband)
        vd = om_d(k);
        k_obs = 0.5;                % Observer gain (Canudas de Wit 1995, Eq. 12)
        e_pos = th_d(k) - th_meas;  % Position tracking error
        
        % Standstill Anti-Hunting Deadband (Eliminates torque buzzing at rest)
        if abs(vd) < 1e-4 && abs(e_pos) < 5e-4
            e_obs = 0.0;
        else
            e_obs = e_pos;
        end

        switch method
            case 'No Comp (PD)'
                T_fric = 0.0;
                
            case 'Coulomb+Viscous'
                % Note: tanh avoids numerical discontinuities at exactly zero velocity
                T_fric = p.Fc * tanh(vd / 1e-4) + p.sigma2 * vd;
                
            case 'LuGre (Truth)'
                gv_d   = p.Fc + (p.Fs - p.Fc) * exp(-(vd / p.vs)^2);
                beta_l = (p.sigma0 * abs(vd)) / gv_d;
                if beta_l > 1e-6
                    z_ss_l = (vd + k_obs * e_obs) / beta_l;
                    z_ctrl = z_ss_l + (z_ctrl - z_ss_l) * exp(-beta_l * Ts);
                else
                    z_ctrl = z_ctrl + Ts * (vd + k_obs * e_obs);
                end
                dz_c = vd - beta_l * z_ctrl + k_obs * e_obs;
                T_fric = p.sigma0 * z_ctrl + p.sigma1 * dz_c + p.sigma2 * vd;
            
            case 'Model 15 (Proposed)'
                % PURE ALGEBRAIC BACKWARD EULER WITH OBSERVER (Zero exp, unconditionally stable!)
                denom_s = p.K * (vd^2) + p.C2;
                beta_s  = abs(vd) * p.S0 * (1 - 1 / denom_s);
                
                % Single-line closed-form algebraic solve:
                z_ctrl  = (z_ctrl + Ts * (vd + k_obs * e_obs)) / (1 + beta_s * Ts);
                
                dz_c    = vd - beta_s * z_ctrl + k_obs * e_obs;
                T_fric  = p.sigma0 * z_ctrl + p.sigma1 * dz_c + p.sigma2 * vd;
        end
        
        % Total Commanded Control Torque
        T_star = T_fb + T_ff + T_fric;
        
        % Save logged states
        th_arr(k)   = th;
        om_arr(k)   = om;
        T_cmd(k)    = T_star;
        T_fb_arr(k) = T_fb;
        
        % 4. Physical Plant Integration over Ts (Analytical 100 kHz ETD Sub-stepping)
        for s = 1:n_sub
            % Physical LuGre friction calculation
            gv = p.Fc + (p.Fs - p.Fc) * exp(-(om / p.vs)^2);
            a = (p.sigma0 * abs(om)) / gv;
            z_ss = (gv / p.sigma0) * sign(om);
            if a > 1e-6
                z_true = z_ss + (z_true - z_ss) * exp(-a * dt);
            else
                z_true = z_true + dt * om;
            end
            dz_true = om - a * z_true;
            F_fric = p.sigma0 * z_true + p.sigma1 * dz_true + p.sigma2 * om;
            
            % Mechanical Dynamics (with lumped hardware disturbances)
            if p.use_noise
                tau_dist = p.tau_cog_amp * sin(12 * th) + p.tau_adc_std * randn();
            else
                tau_dist = 0.0;
            end
            
            d_om = ((T_star + tau_dist) - F_fric) / p.J;
            om = om + d_om * dt;
            th = th + om * dt;
        end
    end
    
    res.th   = th_arr;
    res.om   = om_arr;
    res.e    = th_d - th_arr;
    res.T_cmd = T_cmd;
    res.T_fb  = T_fb_arr;
end