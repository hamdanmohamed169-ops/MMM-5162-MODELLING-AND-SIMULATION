%% MMM 5162 - Modelling and Simulation, Fall 2026 (Term 481)
%  Mini Project 2 - Track MR: Vertical Drone Motion with Thrust-Actuator Lag
%  Student number S = 19
%
%  State model:
%     zdot = v
%     vdot = (T - m*g - c*v*|v|) / m
%     Tdot = (Tcmd(t) - T) / tau_T
%
%  Tcmd(t) = r*m*g for 1 s <= t <= 1+pulse_duration, else m*g
%  Initial conditions: z(0)=0, v(0)=0, T(0)=m*g

clear; clc; close all;

%% 1. Student-specific parameters (S = 19)
S = 19;
g = 9.81;                          % m/s^2
m = 1.60 + 0.03*S;                 % kg
c = 0.120 + 0.005*S;               % quadratic drag coefficient
tau_T = 0.160 + 0.005*S;           % thrust actuator time constant, s
r = 1.180 + 0.005*S;               % pulse thrust ratio
pulse_start = 1.0;                 % s
pulse_duration = 1.40 + 0.03*S;    % s
pulse_end = pulse_start + pulse_duration;
tspan = [0 8];                     % simulation interval, s

fprintf('=== Student-specific parameters (S = %d) ===\n', S);
fprintf('m      = %.4f kg\n', m);
fprintf('c      = %.4f N/(m/s)^2\n', c);
fprintf('tau_T  = %.4f s\n', tau_T);
fprintf('r      = %.4f\n', r);
fprintf('pulse_duration = %.4f s (active %.2f-%.2f s)\n', pulse_duration, pulse_start, pulse_end);
fprintf('Hover thrust m*g = %.4f N, Pulse thrust r*m*g = %.4f N\n\n', m*g, r*m*g);

params = struct('g', g, 'm', m, 'c', c, 'tau_T', tau_T, 'r', r, ...
                 'pulse_start', pulse_start, 'pulse_end', pulse_end);

x0 = [0; 0; m*g];   % [z0; v0; T0]

%% 2. Standard ODE solver (ode45, tight tolerances -> reference solution)
opts = odeset('RelTol', 1e-10, 'AbsTol', 1e-12, 'MaxStep', 0.01);
t_eval = linspace(tspan(1), tspan(2), 4001);
[t_ref, x_ref] = ode45(@(t,x) drone_ode(t, x, params), t_eval, x0, opts);
z_ref = x_ref(:,1); v_ref = x_ref(:,2); T_ref = x_ref(:,3);

[v_max_ref, idx] = max(v_ref);
t_vmax_ref = t_ref(idx);
z_end_ref = z_ref(end);

fprintf('=== Reference solution (ode45, tight tolerance) ===\n');
fprintf('Max climb velocity  v_max = %.5f m/s at t = %.3f s\n', v_max_ref, t_vmax_ref);
fprintf('Altitude at t=8 s   z(8)  = %.5f m\n\n', z_end_ref);

%% 3. Forward Euler implementation (same state model)
euler_steps = [0.02, 0.005, 0.001];
euler_results = struct();
for i = 1:length(euler_steps)
    dt = euler_steps(i);
    [t_e, z_e, v_e, T_e] = forward_euler(@drone_ode, x0, tspan, dt, params);
    [v_max_e, idxe] = max(v_e);
    t_vmax_e = t_e(idxe);
    z_end_e = z_e(end);
    fname = sprintf('dt_%d', round(dt*1000));
    euler_results.(fname) = struct('dt', dt, 't', t_e, 'z', z_e, 'v', v_e, 'T', T_e, ...
                                    'v_max', v_max_e, 't_vmax', t_vmax_e, 'z_end', z_end_e);
    fprintf('=== Forward Euler, dt = %.3f s ===\n', dt);
    fprintf('Max climb velocity  v_max = %.5f m/s at t = %.3f s (err %+ .5f m/s)\n', ...
            v_max_e, t_vmax_e, v_max_e - v_max_ref);
    fprintf('Altitude at t=8 s   z(8)  = %.5f m (err %+ .5f m)\n\n', z_end_e, z_end_e - z_end_ref);
end

%% 4. Numerical verification table
fprintf('=== Verification table ===\n');
fprintf('%-22s %-8s %-12s %-11s %-10s %-10s %-10s\n', 'Method','dt(s)','v_max(m/s)','t_vmax(s)','z(8)(m)','v_max err','z(8) err');
fprintf('%-22s %-8s %-12.5f %-11.3f %-10.5f %-10.5f %-10.5f\n', 'ode45 (ref.)','-', v_max_ref, t_vmax_ref, z_end_ref, 0, 0);
fn = fieldnames(euler_results);
for i = 1:length(fn)
    R = euler_results.(fn{i});
    fprintf('%-22s %-8.3f %-12.5f %-11.3f %-10.5f %-+10.5f %-+10.5f\n', 'Forward Euler', R.dt, ...
            R.v_max, R.t_vmax, R.z_end, R.v_max - v_max_ref, R.z_end - z_end_ref);
end

%% 5. Automatic sanity check
assert(min(t_ref) >= tspan(1)-1e-9 && max(t_ref) <= tspan(2)+1e-9, 'Time outside interval!');
assert(all(T_ref >= 0), 'Non-physical negative thrust!');
fprintf('\nAutomatic check passed: time within [0,8] s and thrust remains non-negative.\n');

%% 6. Plots
% Figure 1: main transient response
figure('Position',[100 100 700 800]);
subplot(3,1,1);
plot(t_ref, z_ref, 'b-', 'LineWidth', 1.8); grid on;
ylabel('Altitude z (m)'); title(sprintf('Drone Vertical Response (S=%d): Altitude, Velocity, Thrust vs Time', S));

subplot(3,1,2);
plot(t_ref, v_ref, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.8); hold on;
yline(v_max_ref, 'k--', sprintf('max v = %.3f m/s', v_max_ref));
ylabel('Velocity v (m/s)'); grid on;

subplot(3,1,3);
plot(t_ref, T_ref, 'g-', 'LineWidth', 1.8); hold on;
yline(m*g, 'k:', 'hover thrust'); yline(r*m*g, 'r:', 'pulse thrust');
ylabel('Thrust T (N)'); xlabel('Time t (s)'); grid on;
saveas(gcf, 'fig1_transient_response_matlab.png');

% Figure 2: standard solver vs forward Euler
figure('Position',[100 100 750 450]);
plot(t_ref, v_ref, 'k-', 'LineWidth', 2.2, 'DisplayName', 'ode45 (reference)'); hold on;
colors = {[0.8 0 0], [0.9 0.5 0], [0.5 0 0.7]};
for i = 1:length(fn)
    R = euler_results.(fn{i});
    plot(R.t, R.v, 'LineWidth', 1.1, 'Color', colors{i}, ...
         'DisplayName', sprintf('Forward Euler, dt=%.3f s', R.dt));
end
xlabel('Time t (s)'); ylabel('Velocity v (m/s)');
title(sprintf('Standard Solver (ode45) vs Forward Euler: Vertical Velocity (S=%d)', S));
legend('Location','northeast'); grid on;
saveas(gcf, 'fig2_euler_vs_ode45_matlab.png');

% Figure 3: convergence plot
figure('Position',[100 100 700 450]);
dts = euler_steps;
vmax_err = zeros(size(dts)); zend_err = zeros(size(dts));
for i = 1:length(fn)
    R = euler_results.(fn{i});
    vmax_err(i) = abs(R.v_max - v_max_ref);
    zend_err(i) = abs(R.z_end - z_end_ref);
end
loglog(dts, vmax_err, 'o-', 'Color', [0.8 0 0], 'LineWidth', 1.5, 'DisplayName', '|error| in v_{max}'); hold on;
loglog(dts, zend_err, 's-', 'Color', [0 0.3 0.8], 'LineWidth', 1.5, 'DisplayName', '|error| in z(8)');
xlabel('Euler time step dt (s)'); ylabel('Absolute error vs ode45 reference');
title(sprintf('Forward Euler Convergence (S=%d)', S));
legend('Location','northwest'); grid on;
saveas(gcf, 'fig3_euler_convergence_matlab.png');

fprintf('\nPlots saved as PNG files.\n');

%% ------------------------------------------------------------------
function dxdt = drone_ode(t, x, p)
    % First-order state model: x = [z; v; T]
    z = x(1); v = x(2); T = x(3); %#ok<NASGU>
    Tc = Tcmd(t, p);
    zdot = v;
    vdot = (T - p.m*p.g - p.c*v*abs(v)) / p.m;
    Tdot = (Tc - T) / p.tau_T;
    dxdt = [zdot; vdot; Tdot];
end

function Tc = Tcmd(t, p)
    % Commanded thrust: hover thrust except during the climb pulse
    if t >= p.pulse_start && t <= p.pulse_end
        Tc = p.r * p.m * p.g;
    else
        Tc = p.m * p.g;
    end
end

function [t, z, v, T] = forward_euler(odefun, x0, tspan, dt, p)
    n = round((tspan(2)-tspan(1))/dt) + 1;
    t = linspace(tspan(1), tspan(1) + (n-1)*dt, n)';
    x = zeros(n, 3);
    x(1,:) = x0';
    for k = 1:n-1
        dx = odefun(t(k), x(k,:)', p);
        x(k+1,:) = x(k,:) + dt * dx';
    end
    z = x(:,1); v = x(:,2); T = x(:,3);
end
