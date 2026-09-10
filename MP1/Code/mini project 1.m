%% MMM 5162 - Modelling and Simulation - Mini Project 1
% Track MR - Two-Link Robot Pick-Cycle Kinematic Simulation
% Student Parameter: S = 19
%
% This single MATLAB file includes:
% 1. Baseline simulation
% 2. Required parameter study: 0.8T, T, 1.2T
% 3. End-effector path
% 4. Joint angles and velocities
% 5. Cycle-time comparison
% 6. Performance measures
% 7. Automatic model/code checks
% 8. Performance table
%
% No additional MATLAB files are required.

clear;
clc;
close all;

%% ============================================================
% 1. STUDENT-SPECIFIC PARAMETERS
% =============================================================

S = 19;

% Link lengths
L1 = 0.380 + 0.005*S;       % [m]
L2 = 0.300 + 0.004*S;       % [m]

% Joint angles
q1_start = 20 + S;          % [deg]
q1_end   = 75 - 0.5*S;      % [deg]

q2_start = -55 + 0.8*S;     % [deg]
q2_end   = 15 + 0.5*S;      % [deg]

% Nominal cycle time
T_nom = 2.40 + 0.04*S;      % [s]

%% Display parameters

fprintf('=============================================\n');
fprintf(' MMM 5162 - Mini Project 1\n');
fprintf(' Two-Link Robot Pick-Cycle Simulation\n');
fprintf('=============================================\n\n');

fprintf('Student parameter: S = %d\n\n', S);

fprintf('Robot Parameters:\n');
fprintf('L1        = %.4f m\n', L1);
fprintf('L2        = %.4f m\n', L2);
fprintf('q1_start  = %.2f deg\n', q1_start);
fprintf('q1_end    = %.2f deg\n', q1_end);
fprintf('q2_start  = %.2f deg\n', q2_start);
fprintf('q2_end    = %.2f deg\n', q2_end);
fprintf('T_nom     = %.4f s\n\n', T_nom);

%% ============================================================
% 2. BASELINE SIMULATION
% =============================================================

N = 1000;

% Time vector from 0 to nominal cycle time
t_base = linspace(0, T_nom, N);

% Run the two-link robot model
base = two_link_pick_cycle( ...
    t_base, ...
    T_nom, ...
    L1, ...
    L2, ...
    q1_start, ...
    q1_end, ...
    q2_start, ...
    q2_end);

%% ============================================================
% 3. PARAMETER STUDY
% =============================================================
%
% Required:
% 0.8T
% T
% 1.2T

factors = [0.8 1.0 1.2];

labels = { ...
    '0.8T', ...
    '1.0T (Nominal)', ...
    '1.2T'};

results = cell(3,1);

for i = 1:3

    % New cycle time
    T_case = factors(i)*T_nom;

    % New time vector
    t_case = linspace(0, T_case, N);

    % Run simulation
    results{i} = two_link_pick_cycle( ...
        t_case, ...
        T_case, ...
        L1, ...
        L2, ...
        q1_start, ...
        q1_end, ...
        q2_start, ...
        q2_end);

    % Store cycle time
    results{i}.T = T_case;

end

%% ============================================================
% 4. PERFORMANCE MEASURES
% =============================================================

fprintf('\n');
fprintf('===============================================================\n');
fprintf(' PERFORMANCE MEASURES\n');
fprintf('===============================================================\n');

fprintf('%-18s %8s %14s %14s %14s %12s %12s\n', ...
    'Case', ...
    'T [s]', ...
    'max|q1dot|', ...
    'max|q2dot|', ...
    'Path [m]', ...
    'x_end [m]', ...
    'y_end [m]');

fprintf('---------------------------------------------------------------\n');

% Matrix to store results
table_rows = zeros(3,6);

for i = 1:3

    r = results{i};

    % Maximum absolute joint velocities
    max_q1dot = max(abs(r.q1dot));
    max_q2dot = max(abs(r.q2dot));

    % End-effector path length
    PL = path_length(r.x, r.y);

    % Final end-effector position
    x_end = r.x(end);
    y_end = r.y(end);

    % Display results
    fprintf('%-18s %8.3f %14.4f %14.4f %14.4f %12.4f %12.4f\n', ...
        labels{i}, ...
        r.T, ...
        max_q1dot, ...
        max_q2dot, ...
        PL, ...
        x_end, ...
        y_end);

    % Save results
    table_rows(i,:) = [ ...
        r.T, ...
        max_q1dot, ...
        max_q2dot, ...
        PL, ...
        x_end, ...
        y_end];

end

%% ============================================================
% 5. CREATE PERFORMANCE TABLE
% =============================================================

PerformanceTable = table( ...
    labels', ...
    table_rows(:,1), ...
    table_rows(:,2), ...
    table_rows(:,3), ...
    table_rows(:,4), ...
    table_rows(:,5), ...
    table_rows(:,6), ...
    'VariableNames', { ...
    'Case', ...
    'T_s', ...
    'MaxAbs_q1dot_rad_s', ...
    'MaxAbs_q2dot_rad_s', ...
    'PathLength_m', ...
    'x_end_m', ...
    'y_end_m'});

fprintf('\n');
disp(PerformanceTable);

% Save table as CSV
writetable(PerformanceTable, 'performance_table.csv');

fprintf('Performance table saved as:\n');
fprintf('performance_table.csv\n');

%% ============================================================
% 6. AUTOMATIC MODEL/CODE CHECKS
% =============================================================

fprintf('\n');
fprintf('===============================================================\n');
fprintf(' AUTOMATIC MODEL/CODE CHECKS\n');
fprintf('===============================================================\n');

%% Check 1: Time outside [0,T]

try

    test_time = [-0.5 1.0];

    two_link_pick_cycle( ...
        test_time, ...
        T_nom, ...
        L1, ...
        L2, ...
        q1_start, ...
        q1_end, ...
        q2_start, ...
        q2_end);

    fprintf('Check 1 FAILED.\n');

catch ME

    fprintf('Check 1 PASSED:\n');
    fprintf('Out-of-range time was rejected correctly.\n');
    fprintf('Message: %s\n\n', ME.message);

end

%% Check 2: Negative cycle time

try

    two_link_pick_cycle( ...
        t_base, ...
        -1.0, ...
        L1, ...
        L2, ...
        q1_start, ...
        q1_end, ...
        q2_start, ...
        q2_end);

    fprintf('Check 2 FAILED.\n');

catch ME

    fprintf('Check 2 PASSED:\n');
    fprintf('Negative cycle time was rejected correctly.\n');
    fprintf('Message: %s\n\n', ME.message);

end

%% Check 3: Zero velocity at start and end

start_velocity = ...
    abs(base.q1dot(1)) + abs(base.q2dot(1));

end_velocity = ...
    abs(base.q1dot(end)) + abs(base.q2dot(end));

if start_velocity < 1e-10 && end_velocity < 1e-10

    fprintf('Check 3 PASSED:\n');
    fprintf('Joint velocities are zero at the start and end.\n\n');

else

    fprintf('Check 3 FAILED:\n');
    fprintf('Joint velocities are not zero at both ends.\n\n');

end

%% ============================================================
% 7. PLOT 1 - END-EFFECTOR PATH
% =============================================================

figure('Name','Plot 1 - End-Effector Path');

plot(base.x, base.y, 'LineWidth', 2);

hold on;

% Start point
plot( ...
    base.x(1), ...
    base.y(1), ...
    'go', ...
    'MarkerSize', 8, ...
    'LineWidth', 2);

% End point
plot( ...
    base.x(end), ...
    base.y(end), ...
    'rs', ...
    'MarkerSize', 8, ...
    'LineWidth', 2);

xlabel('x [m]');
ylabel('y [m]');

title(sprintf( ...
    'End-Effector Path (T = %.2f s)', ...
    T_nom));

axis equal;
grid on;

legend( ...
    'End-Effector Path', ...
    'Start', ...
    'End', ...
    'Location', ...
    'best');

saveas(gcf, 'plot1_ee_path.png');

%% ============================================================
% 8. PLOT 2 - JOINT ANGLES AND VELOCITIES
% =============================================================

figure('Name','Plot 2 - Joint Motion');

% ---------------- Joint Angles ----------------

subplot(2,1,1);

plot( ...
    base.t, ...
    rad2deg(base.q1), ...
    'LineWidth', 1.8);

hold on;

plot( ...
    base.t, ...
    rad2deg(base.q2), ...
    'LineWidth', 1.8);

xlabel('Time [s]');
ylabel('Joint Angle [deg]');

title('Baseline Joint Angles');

legend( ...
    'q_1(t)', ...
    'q_2(t)', ...
    'Location', ...
    'best');

grid on;

% ---------------- Joint Velocities ----------------

subplot(2,1,2);

plot( ...
    base.t, ...
    base.q1dot, ...
    'LineWidth', 1.8);

hold on;

plot( ...
    base.t, ...
    base.q2dot, ...
    'LineWidth', 1.8);

xlabel('Time [s]');
ylabel('Angular Velocity [rad/s]');

title('Baseline Joint Velocities');

legend( ...
    'q_1 dot', ...
    'q_2 dot', ...
    'Location', ...
    'best');

grid on;

saveas(gcf, 'plot2_joint_motion.png');

%% ============================================================
% 9. PLOT 3 - PARAMETER STUDY COMPARISON
% =============================================================

figure('Name','Plot 3 - Cycle Time Comparison');

% ---------------- Joint 1 ----------------

subplot(1,2,1);

hold on;

for i = 1:3

    r = results{i};

    plot( ...
        r.t, ...
        r.q1dot, ...
        'LineWidth', 1.6, ...
        'DisplayName', ...
        sprintf('%s, T = %.2f s', ...
        labels{i}, r.T));

end

xlabel('Time [s]');
ylabel('q_1 dot [rad/s]');

title('Joint-1 Speed Comparison');

grid on;

legend('Location','best');

% ---------------- Joint 2 ----------------

subplot(1,2,2);

hold on;

for i = 1:3

    r = results{i};

    plot( ...
        r.t, ...
        r.q2dot, ...
        'LineWidth', 1.6, ...
        'DisplayName', ...
        sprintf('%s, T = %.2f s', ...
        labels{i}, r.T));

end

xlabel('Time [s]');
ylabel('q_2 dot [rad/s]');

title('Joint-2 Speed Comparison');

grid on;

legend('Location','best');

saveas(gcf, 'plot3_speed_comparison.png');

%% ============================================================
% 10. FINAL SUMMARY
% =============================================================

fprintf('\n');
fprintf('===============================================================\n');
fprintf(' SIMULATION COMPLETED SUCCESSFULLY\n');
fprintf('===============================================================\n');

fprintf('\nGenerated files:\n');
fprintf('1. plot1_ee_path.png\n');
fprintf('2. plot2_joint_motion.png\n');
fprintf('3. plot3_speed_comparison.png\n');
fprintf('4. performance_table.csv\n');

fprintf('\nEngineering Conclusion:\n');
fprintf(['Reducing the cycle time increases the required peak joint ' ...
         'angular velocity, while increasing the cycle time reduces ' ...
         'the required joint speed. The end-effector path remains ' ...
         'unchanged because the link lengths and initial/final joint ' ...
         'angles are kept constant.\n']);

%% ============================================================
% LOCAL FUNCTION 1: TWO-LINK ROBOT MODEL
% =============================================================

function result = two_link_pick_cycle( ...
    t, T, L1, L2, ...
    q1_start, q1_end, ...
    q2_start, q2_end)

    %% Input validation

    if T <= 0
        error('Cycle time T must be positive.');
    end

    if any(t < 0) || any(t > T)
        error('Time values must be within the range [0,T].');
    end

    if L1 <= 0 || L2 <= 0
        error('Link lengths must be positive.');
    end

    %% Normalized time

    u = t ./ T;

    %% Cubic smoothstep

    s = 3*u.^2 - 2*u.^3;

    %% Derivative of smoothstep

    ds_dt = 6*u.*(1-u) ./ T;

    %% Convert angles to radians

    q1_start_rad = deg2rad(q1_start);
    q1_end_rad   = deg2rad(q1_end);

    q2_start_rad = deg2rad(q2_start);
    q2_end_rad   = deg2rad(q2_end);

    %% Joint positions

    q1 = q1_start_rad + ...
        (q1_end_rad - q1_start_rad).*s;

    q2 = q2_start_rad + ...
        (q2_end_rad - q2_start_rad).*s;

    %% Joint angular velocities

    q1dot = ...
        (q1_end_rad - q1_start_rad).*ds_dt;

    q2dot = ...
        (q2_end_rad - q2_start_rad).*ds_dt;

    %% Forward kinematics

    x = L1*cos(q1) + ...
        L2*cos(q1 + q2);

    y = L1*sin(q1) + ...
        L2*sin(q1 + q2);

    %% Store results

    result.t = t;

    result.q1 = q1;
    result.q2 = q2;

    result.q1dot = q1dot;
    result.q2dot = q2dot;

    result.x = x;
    result.y = y;

end

%% ============================================================
% LOCAL FUNCTION 2: PATH LENGTH
% =============================================================

function L = path_length(x, y)

    dx = diff(x);
    dy = diff(y);

    L = sum(sqrt(dx.^2 + dy.^2));

end