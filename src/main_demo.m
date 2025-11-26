%% 3D Missile–Aircraft Pursuit Simulation (Toy IR Heat-Seeker Style)
clear; clc; close all;

%% TIME SETTINGS
dt      = 0.01;            % time step [s]
t_final = 60;              % total sim time [s]
t       = 0:dt:t_final;
N       = numel(t);

%% SPEEDS (roughly realistic order of magnitude)
Va = 250;                  % aircraft speed [m/s]   ~ 900 km/h
Vm = 1.5 * Va;             % missile speed [m/s]    faster than aircraft

t_launch = 5.0;            % missile launch delay [s]

%% MISSILE MANEUVER LIMITS (toy, not real data)
g0        = 9.81;
a_n_max   = 25 * g0;       % max lateral accel [m/s^2] (25 g)
max_turn_rate = a_n_max / Vm;  % [rad/s]
max_dtheta    = max_turn_rate * dt;

%% KILL RADIUS
R_kill = 50;               % [m] distance for "hit"

%% AIRCRAFT TRAJECTORY (3D)
% Timeline:
% 0    - t1: straight, level
% t1   - t2: small maneuvers (yaw + pitch)
% t2   - t2+Tturn: 90° left turn in yaw, slight climb
% >t2+Tturn: straight on new heading

t1          = 8;           % start maneuvers
t2          = 16;          % start 90° turn
turn_duration = 10;        % time to complete 90° turn

psi_a   = zeros(1, N);     % yaw angle [rad]
theta_a = zeros(1, N);     % pitch angle [rad]

for k = 1:N
    tk = t(k);

    if tk < t1
        % straight, level
        psi_a(k)   = 0;
        theta_a(k) = 0;

    elseif tk < t2
        % small maneuvers: weaving + small climb/dive
        psi_a(k)   = deg2rad(5) * sin(2*pi*(tk - t1)/4);   % ±5° yaw
        theta_a(k) = deg2rad(3) * sin(2*pi*(tk - t1)/5);   % ±3° pitch

    elseif tk < t2 + turn_duration
        % 90° left turn (yaw from 0 -> +90°), gentle climb
        frac       = (tk - t2) / turn_duration;
        psi_a(k)   = frac * deg2rad(90);                   % 0 -> 90°
        theta_a(k) = deg2rad(2) * sin(pi * frac);          % little bump in pitch

    else
        % after turn: straight on +90° heading, level
        psi_a(k)   = deg2rad(90);
        theta_a(k) = 0;
    end
end

% Integrate aircraft position
pos_a = zeros(3, N);       % [x; y; z]
pos_a(:,1) = [0; 0; 0];    % start at origin

for k = 2:N
    psi   = psi_a(k-1);
    theta = theta_a(k-1);

    % Direction unit vector (3D)
    u_ax = cos(theta) * cos(psi);
    u_ay = cos(theta) * sin(psi);
    u_az = sin(theta);
    u_a  = [u_ax; u_ay; u_az];

    pos_a(:,k) = pos_a(:,k-1) + Va * u_a * dt;
end

%% MISSILE INITIAL CONDITIONS
pos_m = zeros(3, N);
vel_m = zeros(3, N);
u_m   = zeros(3, N);       % direction unit vector

% Start behind, offset, and below the aircraft
pos_m(:,1) = [-4000; -1500; -800];

% Initial aim: towards aircraft initial position
r0      = pos_a(:,1) - pos_m(:,1);
u_m(:,1) = r0 / norm(r0);
vel_m(:,1) = Vm * u_m(:,1);

%% TOY IR SEEKER MODEL (direction tracking with noise + lag)
seeker_dir = u_m(:,1);         % initial seeker line-of-sight
noise_std  = 0.01;             % small angular-ish noise
alpha      = 0.9;              % lag factor (close to 1 => more lag)

hit       = false;
hit_index = NaN;

for k = 2:N
    tk = t(k);

    % Missile sits idle before launch
    if tk < t_launch
        pos_m(:,k) = pos_m(:,k-1);
        vel_m(:,k) = [0; 0; 0];
        u_m(:,k)   = u_m(:,k-1);
        continue;
    end

    % Relative vector to aircraft (heat source position)
    r = pos_a(:,k-1) - pos_m(:,k-1);
    dist = norm(r);

    % Kill check
    if dist < R_kill && ~hit
        hit       = true;
        hit_index = k-1;
        pos_m(:,k) = pos_m(:,k-1);
        vel_m(:,k) = vel_m(:,k-1);
        u_m(:,k)   = u_m(:,k-1);
        break;
    end

    if dist < 1e-6
        % Degenerate, just bail
        hit       = true;
        hit_index = k-1;
        break;
    end

    % Line-of-sight unit vector (true)
    u_LOS = r / dist;

    % Add small noise (representing imperfect IR direction measurement)
    meas = u_LOS + noise_std * randn(3,1);
    meas = meas / norm(meas);

    % Apply seeker lag (first-order filter on direction)
    seeker_dir = alpha * seeker_dir + (1 - alpha) * meas;
    seeker_dir = seeker_dir / norm(seeker_dir);

    % Rotate missile direction u_m(:,k-1) toward seeker_dir
    u_old = u_m(:,k-1);
    cosang = dot(u_old, seeker_dir);
    cosang = max(min(cosang, 1), -1);     % clamp
    ang = acos(cosang);                    % angle between

    if ang < 1e-6
        u_new = u_old;                     % already aligned
    else
        dtheta = min(ang, max_dtheta);     % limit rotation this step

        % Rotation axis (unit)
        axis = cross(u_old, seeker_dir);
        na   = norm(axis);
        if na < 1e-9
            % If axis is poorly defined, just keep direction
            u_new = u_old;
        else
            axis = axis / na;

            % Rodrigues' rotation formula: rotate u_old by dtheta around axis
            u_new = u_old * cos(dtheta) + ...
                    cross(axis, u_old) * sin(dtheta) + ...
                    axis * (dot(axis, u_old)) * (1 - cos(dtheta));
        end
    end

    % Normalize for safety
    u_new = u_new / norm(u_new);

    u_m(:,k)   = u_new;
    vel_m(:,k) = Vm * u_new;
    pos_m(:,k) = pos_m(:,k-1) + vel_m(:,k) * dt;
end

% Trim if hit early
if hit
    pos_a = pos_a(:,1:hit_index);
    pos_m = pos_m(:,1:hit_index);
    t     = t(1:hit_index);
end

%% 3D PLOT
figure; hold on; grid on; axis equal;
plot3(pos_a(1,:), pos_a(2,:), pos_a(3,:), 'r', 'LineWidth', 2);   % aircraft red
plot3(pos_m(1,:), pos_m(2,:), pos_m(3,:), 'k', 'LineWidth', 2);   % missile black

plot3(pos_a(1,1), pos_a(2,1), pos_a(3,1), 'ro', 'MarkerFaceColor', 'r'); % aircraft start
plot3(pos_m(1,1), pos_m(2,1), pos_m(3,1), 'ko', 'MarkerFaceColor', 'k'); % missile start

if hit
    plot3(pos_m(1,end), pos_m(2,end), pos_m(3,end), 'kx', 'MarkerSize', 12, 'LineWidth', 2);
    legend('Aircraft (red)', 'Missile (black)', 'Aircraft start', 'Missile start', 'Intercept', ...
           'Location', 'best');
else
    legend('Aircraft (red)', 'Missile (black)', 'Aircraft start', 'Missile start', ...
           'Location', 'best');
end

xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
title('3D Missile–Aircraft Pursuit (Toy IR Heat-Seeker)');
view(40, 25);
