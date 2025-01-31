clc; close all; clear all;

addpath("lib\");
% initialization
init_pos = [-10; -10; 0];
UAV.position = init_pos;
UAV.velocity = [6; 1; 1];
UAV.speed = 7;

WayPoints = [-10, -10, 0;...
             20, 0, 5;...
             20, 20, 5;...
             0, 20, 5;...
             0, 0, 5;...
             20, 0, 5]';
waypoint_index = 2;

Circular1.center = [20;...
                    10;...
                    5];
    
Circular1.lambda_h = 1;
Circular1.rho_h = 10;
spanning_angle = linspace(-pi/2, pi/2, 100);
Circular1_path = [cos(spanning_angle);...
                  sin(spanning_angle);...
                  zeros([1, length(spanning_angle)])]*Circular1.rho_h + Circular1.center;

Circular2.center = [0;...
                    10;...
                    5];
Circular2.lambda_h = 1;
Circular2.rho_h = 10;
spanning_angle = linspace(pi/2, pi/2*3, 100);
Circular2_path = [cos(spanning_angle);...
                  sin(spanning_angle);...
                  zeros([1, length(spanning_angle)])]*Circular2.rho_h + Circular2.center;

sim_time = 20;
sampling_time = 0.1;
time = 0;

simulationTime = linspace(0, sim_time, sampling_time);

begin_point = init_pos;
end_point = WayPoints(:, waypoint_index);

K1 = [1, 0, 0;...
      0, 1, 0;...
      0, 0, 10];
K2 = [4, 0, 0;...
      0, 4, 0;...
      0, 0, 2];


linecolor = [0 0.4470 0.7410];
fig1 = figure(1);
trajectory = [];
ref_command_log = [];
pos_log = [];
%%
frame = 1;
while waypoint_index < 6
    plot3([WayPoints(1,1:2)], [WayPoints(2,1:2)], [WayPoints(3,1:2)], 'o--', 'Color', linecolor);
    hold on
    plot3([WayPoints(1,3:4)], [WayPoints(2,3:4)], [WayPoints(3,3:4)], 'o--', 'Color', linecolor);
    plot3(Circular1_path(1,:), Circular1_path(2,:), Circular1_path(3,:), '--', 'Color', linecolor);
    plot3(Circular2_path(1,:), Circular2_path(2,:), Circular2_path(3,:), '--', 'Color', linecolor);

    arr_dist = norm(end_point - UAV.position);
    if (arr_dist < 1)
        waypoint_index = waypoint_index + 1;
        if waypoint_index > 6
            break
        end
        begin_point = WayPoints(:, waypoint_index-1);
        end_point= WayPoints(:, waypoint_index);
    end
    
    unit_velocity = UAV.velocity./norm(UAV.velocity);
    reference_command = straight_line_vector_field(begin_point, end_point, UAV.position, K1, K2);
    if waypoint_index == 3
        reference_command = circular_line_vector_field(Circular1, UAV.position, K1, K2);
    elseif waypoint_index == 5
        reference_command = circular_line_vector_field(Circular2, UAV.position, K1, K2);
    end
    
    ref_command_log = [ref_command_log, reference_command];
    pos_log = [pos_log, UAV.position];

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
    
    xlabel("E-axis"); ylabel("N-axis"); zlabel("U-axis"); title("Vector-Field Test");
    hold off
    time = time + sampling_time;
    pause(0.01);
    frame = frame + 1;
end
%%

for i = 1 : 18
    idx = 10*i;
    hold on
    scatter3(pos_log(1, idx), pos_log(2, idx), pos_log(3, idx), 'green', 'filled');
    plot3([pos_log(1,idx), pos_log(1,idx) + ref_command_log(1, idx)], [pos_log(2,idx), pos_log(2,idx) + ref_command_log(2, idx)], [pos_log(3,idx), pos_log(3,idx) + ref_command_log(3, idx)], 'r-', 'LineWidth', 0.7)
end
legend([drone, Traj, InputC], {"UAV", "Trajectory", "Control Input"}, 'Location', 'northwest')
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