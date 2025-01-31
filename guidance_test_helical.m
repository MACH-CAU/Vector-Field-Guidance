clc; close all; clear all;

addpath("lib\");

HelicalPath = struct;
HelicalPath.center = [0; 0; 0]; % Center Point
HelicalPath.gamma_h = deg2rad(20);       % Climb angle 10deg
HelicalPath.lambda_h = 1;         % CW
HelicalPath.rho_h = 10;         % Radius of Helical Path
HelicalPath.psi = 0;            % Init waypoint angle of Helical Path (Starts from [10, 0, 0]')
UAV.position = [10; 0; 0];
UAV.velocity = [0; -7; 0];
UAV.speed = 7;

K1 = diag(ones([1, 3])*12);
K2 = diag(ones([1, 3])*40);

trajectory = [];

fig1 = figure(1);

frame = 1;

sim_time = 15;
sampling_time = 0.1;
time = 0;

time_step = linspace(0, sim_time/3*2, sim_time/sampling_time + 1);
path = [HelicalPath.rho_h*cos(HelicalPath.lambda_h*time_step + HelicalPath.psi);...
         HelicalPath.rho_h*sin(HelicalPath.lambda_h*time_step + HelicalPath.psi);...
         time_step*norm(UAV.velocity)*tan(HelicalPath.gamma_h)];

% vid = video_writer("guidance_test_helical", sampling_time);

while time < sim_time
    Path = plot3(path(1, :), path(2, :), path(3, :));
    hold on
    unit_velocity = UAV.velocity./norm(UAV.velocity);
    reference_command = helical_line_vector_field(HelicalPath, UAV.position, K1, K2);
    velocity_err = reference_command - unit_velocity;
    control_input = velocity_err + unit_velocity;
    control_input = control_input./norm(control_input);
    unit_updated = first_order_lag_filter(control_input);
    UAV.velocity = unit_updated.*UAV.speed;

    UAV.position = UAV.position + UAV.velocity*sampling_time;
    
    drone = scatter3(UAV.position(1), UAV.position(2), UAV.position(3), 'green', 'filled');
    trajectory = [trajectory, UAV.position];
    Traj = plot3(trajectory(1, :), trajectory(2, :), trajectory(3, :), 'g-', 'LineWidth', 1);
    InputC = plot3([UAV.position(1), UAV.position(1) + 2*unit_updated(1)], [UAV.position(2), UAV.position(2) + 2*unit_updated(2)], [UAV.position(3), UAV.position(3) + 2*unit_updated(3)], 'r-', 'LineWidth', 0.7);
    grid on
    legend([drone, Traj, InputC], {"UAV", "Trajectory", "Control Input"}, 'Location', 'northwest')
    xlabel("E-axis"); ylabel("N-axis"); zlabel("U-axis"); title("Vector-Field Test");
    hold off
    time = time + sampling_time;
    pause(0.01);
    frame = frame + 1;
    % vid.update(frame*sampling_time);
end
% vid.close();

function output = first_order_lag_filter(input)
    persistent k    % Filter gain k
    persistent Tr   % Maximum response time
    persistent Ts   % Sampling time
    persistent Frac % Desired fraction of change
    persistent old_val

    if isempty(k)
        Tr = 0.02;
        Ts = 0.001;
        Frac = 0.9;

        k = 1 - exp(log(1-Frac)*Ts/Tr);
        output = input;
        old_val = input;
    else
        output = k*input + (1-k)*old_val;
        old_val = output;
    end
end
