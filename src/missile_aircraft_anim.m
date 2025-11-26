%% 3D Missile–Aircraft Pursuit Simulation (Toy IR Heat-Seeker) + ANIMATION
clear; clc; close all;

%% TIME SETTINGS
dt      = 0.01;            % time step [s]
t_final = 60;              % total sim time [s]
t       = 0:dt:t_final;
N       = numel(t);

%% SPEEDS (roughly realistic scale)
Va = 250;                  % aircraft speed [m/s]   ~ 900 km/h
Vm = 1.5 * Va;             % missile speed [m/s]

t_launch = 5.0;            % missile launch delay [s]

%% MISSILE MANEUVER LIMITS (toy, not real data)
g0        = 9.81;
a_n_max   = 25 * g0;       % max lateral accel [m/s^2] (25 g)
max_turn_rate = a_n_max / Vm;  % [rad/s]
max_dtheta    = max_turn_rate * dt;

%% KILL RADIUS
R_kill = 50;               % [m] distance for "hit"

%% AIRCRAFT TRAJECTORY (3D)
% 0    - t1: straight, level
% t1   - t2: small maneuvers (yaw + pitch)
% t2   - t2+Tturn: 90° left turn in yaw, slight climb bump
% >t2+Tturn: straight on new heading

t1            = 8;         % start maneuvers
t2            = 16;        % start 90° turn
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
        % 90° left turn (yaw 0 -> +90°), gentle bump in pitch
        frac       = (tk - t2) / turn_duration;
        psi_a(k)   = frac * deg2rad(90);                   % 0 -> 90°
        theta_a(k) = deg2rad(2) * sin(pi * frac);          % small vertical maneuver

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
r0        = pos_a(:,1) - pos_m(:,1);
u_m(:,1)  = r0 / norm(r0);
vel_m(:,1)= Vm * u_m(:,1);

%% TOY IR SEEKER MODEL (direction tracking with noise + lag)
seeker_dir = u_m(:,1);     % initial seeker line-of-sight
noise_std  = 0.01;         % small direction noise
alpha      = 0.9;          % lag factor (close to 1 => more lag)

hit       = false;
hit_index = N;

%% SIMULATION LOOP
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
    r    = pos_a(:,k-1) - pos_m(:,k-1);
    dist = norm(r);

    % Kill check
    if dist < R_kill && ~hit
        hit        = true;
        hit_index  = k-1;
        pos_m(:,k) = pos_m(:,k-1);
        vel_m(:,k) = vel_m(:,k-1);
        u_m(:,k)   = u_m(:,k-1);
        break;
    end

    if dist < 1e-6
        hit       = true;
        hit_index = k-1;
        break;
    end

    % True line-of-sight direction
    u_LOS = r / dist;

    % Add small noise (imperfect IR measurement)
    meas = u_LOS + noise_std * randn(3,1);
    meas = meas / norm(meas);

    % Apply seeker lag
    seeker_dir = alpha * seeker_dir + (1 - alpha) * meas;
    seeker_dir = seeker_dir / norm(seeker_dir);

    % Rotate missile direction toward seeker_dir with limited angle
    u_old = u_m(:,k-1);
    cosang = dot(u_old, seeker_dir);
    cosang = max(min(cosang, 1), -1);     % clamp
    ang = acos(cosang);

    if ang < 1e-6
        u_new = u_old;
    else
        dtheta = min(ang, max_dtheta);

        % Rotation axis
        axis = cross(u_old, seeker_dir);
        na   = norm(axis);
        if na < 1e-9
            u_new = u_old;
        else
            axis = axis / na;

            % Rodrigues' rotation formula
            u_new = u_old * cos(dtheta) + ...
                    cross(axis, u_old) * sin(dtheta) + ...
                    axis * (dot(axis, u_old)) * (1 - cos(dtheta));
        end
    end

    u_new      = u_new / norm(u_new);
    u_m(:,k)   = u_new;
    vel_m(:,k) = Vm * u_new;
    pos_m(:,k) = pos_m(:,k-1) + vel_m(:,k) * dt;
end

% Trim if hit early
pos_a = pos_a(:,1:hit_index);
pos_m = pos_m(:,1:hit_index);
t     = t(1:hit_index);
N     = numel(t);

%% PREPARE ANIMATION LIMITS
xmin = min([pos_a(1,:) pos_m(1,:)]);
xmax = max([pos_a(1,:) pos_m(1,:)]);
ymin = min([pos_a(2,:) pos_m(2,:) ]);
ymax = max([pos_a(2,:) pos_m(2,:) ]);
zmin = min([pos_a(3,:) pos_m(3,:) ]);
zmax = max([pos_a(3,:) pos_m(3,:) ]);

% Add some margin
margin = 0.1;
dx = xmax - xmin; if dx == 0, dx = 1; end
dy = ymax - ymin; if dy == 0, dy = 1; end
dz = zmax - zmin; if dz == 0, dz = 1; end

xmin = xmin - margin*dx; xmax = xmax + margin*dx;
ymin = ymin - margin*dy; ymax = ymax + margin*dy;
zmin = zmin - margin*dz; zmax = zmax + margin*dz;

%% 3D ANIMATION
figure;
hold on; grid on; axis equal;
xlim([xmin xmax]);
ylim([ymin ymax]);
zlim([zmin zmax]);
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
title('3D Missile–Aircraft Pursuit (Toy IR Heat-Seeker)');
view(40, 25);

% Static legend objects
hA_traj = plot3(pos_a(1,1), pos_a(2,1), pos_a(3,1), 'r', 'LineWidth', 2);   % aircraft trail
hM_traj = plot3(pos_m(1,1), pos_m(2,1), pos_m(3,1), 'k', 'LineWidth', 2);   % missile trail

hA_pt = plot3(pos_a(1,1), pos_a(2,1), pos_a(3,1), 'ro', 'MarkerFaceColor', 'r'); % aircraft point
hM_pt = plot3(pos_m(1,1), pos_m(2,1), pos_m(3,1), 'ko', 'MarkerFaceColor', 'k'); % missile point

legend('Aircraft (red)', 'Missile (black)', 'Aircraft position', 'Missile position', ...
       'Location', 'bestoutside');

% Animation loop
for k = 1:N
    % Update trajectories
    set(hA_traj, 'XData', pos_a(1,1:k), 'YData', pos_a(2,1:k), 'ZData', pos_a(3,1:k));
    set(hM_traj, 'XData', pos_m(1,1:k), 'YData', pos_m(2,1:k), 'ZData', pos_m(3,1:k));

    % Update current points
    set(hA_pt, 'XData', pos_a(1,k), 'YData', pos_a(2,k), 'ZData', pos_a(3,k));
    set(hM_pt, 'XData', pos_m(1,k), 'YData', pos_m(2,k), 'ZData', pos_m(3,k));

    drawnow;  % render this frame
    % If you want slower playback:
    % pause(0.01);
end

% Mark intercept at the end (if missile reached)
if hit
    hold on;
    plot3(pos_m(1,end), pos_m(2,end), pos_m(3,end), 'kx', 'MarkerSize', 12, 'LineWidth', 2);
end
