%% ========================================================================
%   1-DOF ACTUATOR CLOSED-LOOP BENCHMARK SUITE (Pure MATLAB / No Simulink)
%   Control Architecture: 2-DOF Control (PD Feedback + Dynamics Feedforward)
%   Plant: Rotor Inertia (J) + True LuGre Friction Dynamics
%   Comparison: 
%     - Case A: Baseline PD (No compensation)
%     - Case B: PD + Classical Coulomb + Viscous Feedforward
%     - Case C: PD + Full LuGre Feedforward (Upper Benchmark)
%     - Case D: PD + Proposed Symbolic Model 15 Feedforward
% ========================================================================
clearvars; clc; close all;

fprintf('====================================================================\n');
fprintf('   STARTING 1-DOF CLOSED-LOOP ACTUATOR CONTROL EVALUATION           \n');
fprintf('====================================================================\n');

%% --- 1. SYSTEM PARAMETERS ---
% Actuator Inertia & Time Base
p.J  = 0.01;            % Motor + Load Inertia [kg*m^2]
f_ctrl = 1000;          % Control frequency: 1 kHz
Ts = 1 / f_ctrl;        % Control period: 1 ms

% Feedback Controller Gains (PD)
Kp = 200.0;             % Proportional gain [Nm/rad]
Kv = 10.0;              % Derivative gain [Nm*s/rad]

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

%% --- 2. EXECUTION TIME BENCHMARK (Calculation Cost in us) ---
fprintf('\n>>> Benchmarking Model Execution Time (100,000 evaluations)...\n');
v_test = 0.05; z_test = 1e-5;
N_eval = 100000;

% 1. Coulomb + Viscous
tic;
for i = 1:N_eval
    T_cv = p.Fc * sign(v_test) + p.sigma2 * v_test;
end
t_cv = (toc / N_eval) * 1e6;

% 2. Full LuGre Update
tic;
z_l = z_test;
for i = 1:N_eval
    gv = p.Fc + (p.Fs - p.Fc) * exp(-(v_test / p.vs)^2);
    dz_l = v_test - (p.sigma0 * abs(v_test) / gv) * z_l;
    z_l = z_l + Ts * dz_l;
    T_lugre = p.sigma0 * z_l + p.sigma1 * dz_l + p.sigma2 * v_test;
end
t_lugre = (toc / N_eval) * 1e6;

% 3. Model 15 Update
tic;
z_s = z_test;
for i = 1:N_eval
    s0_z = p.S0 * z_s;
    dz_s = v_test - abs(v_test) * (s0_z - (s0_z / (p.K * v_test^2 + p.C2)));
    z_s = z_s + Ts * dz_s;
    T_s15 = p.sigma0 * z_s + p.sigma1 * dz_s + p.sigma2 * v_test;
end
t_s15 = (toc / N_eval) * 1e6;

t_exec = [0.05, t_cv, t_lugre, t_s15]; % [us] (PD alone baseline is approx 0.05 us)
fprintf('   -> Coulomb+Viscous : %.3f us\n', t_cv);
fprintf('   -> LuGre ODE       : %.3f us\n', t_lugre);
fprintf('   -> Model 15        : %.3f us (%.1f%% faster than LuGre)\n', t_s15, (1 - t_s15/t_lugre)*100);

%% ========================================================================
%% TRAJECTORY 1: LOW-VELOCITY ZERO-CROSSING (Sinusoid)
%% ========================================================================
fprintf('\n>>> Simulating Trajectory 1: Low-Velocity Zero-Crossing...\n');
t_end_1 = 20.0;
t_vec1 = 0:Ts:t_end_1;
N1 = length(t_vec1);

% Motion: A = 0.2 rad, omega = 0.5 rad/s -> v_max = 0.1 rad/s
A_sin = 0.2; w_sin = 0.5;
th_d1 = A_sin * sin(w_sin * t_vec1);
om_d1 = A_sin * w_sin * cos(w_sin * t_vec1);
al_d1 = -A_sin * (w_sin^2) * sin(w_sin * t_vec1);

results_t1 = cell(4, 1);
for m_idx = 1:4
    results_t1{m_idx} = run_actuator_sim(t_vec1, th_d1, om_d1, al_d1, methods{m_idx}, p, Kp, Kv, Ts);
end

% --- Plot Fig 1: Trajectory 1 Subplots ---
fig1 = figure('Name', 'Trajectory 1: Zero-Crossing Test', 'Color', 'w', 'Position', [50, 50, 950, 750]);

subplot(3, 1, 1); hold on; grid on;
plot(t_vec1, th_d1, 'k--', 'LineWidth', 1.2);
for m_idx = 1:4
    plot(t_vec1, results_t1{m_idx}.th, 'Color', c_map{m_idx}, 'LineWidth', 1.2);
end
ylabel('\theta [rad]'); title('Low-Velocity Reversal Tracking: \theta_d(t) vs. \theta(t)');
legend([{'Desired \theta_d'}, methods], 'Location', 'northeast');

subplot(3, 1, 2); hold on; grid on;
for m_idx = 1:4
    plot(t_vec1, results_t1{m_idx}.e, 'Color', c_map{m_idx}, 'LineWidth', 1.2);
end
ylabel('Error e(t) [rad]'); title('Tracking Error e(t) = \theta_d - \theta');

subplot(3, 1, 3); hold on; grid on;
for m_idx = 1:4
    plot(t_vec1, results_t1{m_idx}.T_cmd, 'Color', c_map{m_idx}, 'LineWidth', 1.1);
end
xlabel('Time [s]'); ylabel('Torque T^* [Nm]'); title('Commanded Torque T^*(t)');

%% ========================================================================
%% TRAJECTORY 2: MULTI-STEP MICRO-POSITIONING (Stick-Slip Suppression)
%% ========================================================================
fprintf('\n>>> Simulating Trajectory 2: Multi-Step Micro-Positioning...\n');
t_end_2 = 12.0;
t_vec2 = 0:Ts:t_end_2;
N2 = length(t_vec2);

th_d2 = zeros(size(t_vec2));
for i = 1:N2
    t_curr = t_vec2(i);
    if t_curr < 2.0
        th_d2(i) = 0.0;
    elseif t_curr < 5.0
        th_d2(i) = 0.01;  % 10 mrad step
    elseif t_curr < 8.0
        th_d2(i) = 0.05;  % 50 mrad step
    else
        th_d2(i) = 0.02;  % Step reversal
    end
end
om_d2 = [0, diff(th_d2)/Ts];
al_d2 = [0, diff(om_d2)/Ts];

results_t2 = cell(4, 1);
for m_idx = 1:4
    results_t2{m_idx} = run_actuator_sim(t_vec2, th_d2, om_d2, al_d2, methods{m_idx}, p, Kp, Kv, Ts);
end

% --- Plot Fig 2: Trajectory 2 Subplots ---
fig2 = figure('Name', 'Trajectory 2: Micro-Step Positioning', 'Color', 'w', 'Position', [100, 80, 950, 750]);

subplot(3, 1, 1); hold on; grid on;
plot(t_vec2, th_d2, 'k--', 'LineWidth', 1.2);
for m_idx = 1:4
    plot(t_vec2, results_t2{m_idx}.th, 'Color', c_map{m_idx}, 'LineWidth', 1.2);
end
ylabel('\theta [rad]'); title('Micro-Step Positioning: Target vs. Actual');
legend([{'Target \theta_d'}, methods], 'Location', 'southeast');

subplot(3, 1, 2); hold on; grid on;
for m_idx = 1:4
    plot(t_vec2, results_t2{m_idx}.e, 'Color', c_map{m_idx}, 'LineWidth', 1.2);
end
ylabel('Error e(t) [rad]'); title('Positioning Error (Steady-State Stick Elimination)');

subplot(3, 1, 3); hold on; grid on;
for m_idx = 1:4
    plot(t_vec2, results_t2{m_idx}.T_cmd, 'Color', c_map{m_idx}, 'LineWidth', 1.1);
end
xlabel('Time [s]'); ylabel('Torque T^* [Nm]'); title('Commanded Torque T^*(t)');

%% ========================================================================
%% --- 3. METRICS COMPUTATION & THESIS TABLE EXPORT ---
%% ========================================================================
rms_err = zeros(4, 1);
max_err = zeros(4, 1);
delay_ms = zeros(4, 1);

% Identify reference zero-crossing points in Trajectory 1
zc_indices = find(diff(sign(om_d1)) ~= 0);

for m_idx = 1:4
    e1 = results_t1{m_idx}.e;
    rms_err(m_idx) = sqrt(mean(e1.^2));
    max_err(m_idx) = max(abs(e1));
    
    % Compute average zero-crossing deadband delay
    delays = [];
    om_act = results_t1{m_idx}.om;
    for zc = zc_indices(2:end-1)
        target_sign = sign(om_d1(zc + 1));
        post_window = zc:(zc + round(0.5/Ts));
        idx_cross = find(sign(om_act(post_window)) == target_sign, 1, 'first');
        if ~isempty(idx_cross)
            delays = [delays, (idx_cross - 1) * Ts * 1000]; % [ms]
        end
    end
    delay_ms(m_idx) = mean(delays);
end

% Construct Thesis Summary Table
T_results = table(methods', rms_err, max_err, delay_ms, t_exec', ...
    'VariableNames', {'Compensation_Method', 'RMS_Error_rad', 'Max_Error_rad', 'ZeroCross_Delay_ms', 'Execution_Time_us'});

fprintf('\n====================================================================\n');
fprintf('                     FINAL THESIS EVALUATION TABLE                 \n');
fprintf('====================================================================\n');
disp(T_results);
fprintf('====================================================================\n');

%% ========================================================================
%% HELPER FUNCTIONS: PLANT INTEGRATOR & CONTROLLER LOOP
%% ========================================================================

function res = run_actuator_sim(t_vec, th_d, om_d, al_d, method, p, Kp, Kv, Ts)
    N = length(t_vec);
    th_arr  = zeros(1, N);
    om_arr  = zeros(1, N);
    T_cmd   = zeros(1, N);
    
    % Initial states
    th = th_d(1);
    om = om_d(1);
    z_true = 0.0;    % Physical plant LuGre state
    z_ctrl = 0.0;    % Controller feedforward internal state
    
    % Sub-stepping parameters (10 substeps per control step to prevent stiffness issues)
    n_sub = 10;
    dt = Ts / n_sub;
    
    for k = 1:N
        % 1. Feedback Torque (PD)
        T_fb = Kp * (th_d(k) - th) + Kv * (om_d(k) - om);
        
        % 2. Inertial Feedforward (Acceleration only; linear viscous damping omitted)
        T_ff = p.J * al_d(k);
        
        % 3. Friction Feedforward (Unified Model)
        vd = om_d(k);
        switch method
            case 'No Comp (PD)'
                T_fric = 0.0;
                
            case 'Coulomb+Viscous'
                % Note: tanh avoids numerical discontinuities at exactly zero velocity
                T_fric = p.Fc * tanh(vd / 1e-4) + p.sigma2 * vd;
                
            case 'LuGre (Truth)'
                gv_d = p.Fc + (p.Fs - p.Fc) * exp(-(vd / p.vs)^2);
                dz_c = vd - (p.sigma0 * abs(vd) / gv_d) * z_ctrl;
                z_ctrl = z_ctrl + Ts * dz_c;
                T_fric = p.sigma0 * z_ctrl + p.sigma1 * dz_c + p.sigma2 * vd;
                
            case 'Model 15 (Proposed)'
                s0_z = p.S0 * z_ctrl;
                dz_c = vd - abs(vd) * (s0_z - (s0_z / (p.K * (vd^2) + p.C2)));
                z_ctrl = z_ctrl + Ts * dz_c;
                T_fric = p.sigma0 * z_ctrl + p.sigma1 * dz_c + p.sigma2 * vd;
        end
        
        % Total Commanded Control Torque
        T_star = T_fb + T_ff + T_fric;
        
        % Save logged states
        th_arr(k) = th;
        om_arr(k) = om;
        T_cmd(k)  = T_star;
        
        % 4. Physical Plant Integration over Ts (Analytical sub-stepping)
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
            
            % Mechanical Dynamics
            d_om = (T_star - F_fric) / p.J;
            om = om + d_om * dt;
            th = th + om * dt;
        end
    end
    
    res.th    = th_arr;
    res.om    = om_arr;
    res.e     = th_d - th_arr;
    res.T_cmd = T_cmd;
end