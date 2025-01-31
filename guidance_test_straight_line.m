clc; close all; clear all;

addpath("lib\");

% initialization
init_pos = [-10; -10; 0];
UAV.position = init_pos;
UAV.velocity = [6; 1; 1];
UAV.speed = 7;

WayPoints = [-10, -10, 0;...
             20, 0, 10;...
             20, 20, 10;...
             0, 20, 10;...
             0, 0, 10]';
waypoint_index = 2;

sim_time = 20;
sampling_time = 0.1;
time = 0;

simulationTime = linspace(0, sim_time, sampling_time);

begin_point = init_pos;
end_point = WayPoints(:, waypoint_index);

K1 = [3, .1, .1;...
      .1, 3, .1;...
      .1, .1, 3];
K2 = [6, .1, .1;...
      .1, 6, .1;...
      .1, .1, 6];

trajectory = [];

filename = 'test.gif';
fig1 = figure(1);       

frame_idx = 1;
while waypoint_index < 6
    arr_dist = norm(end_point - UAV.position);
    if (arr_dist < 1)
        waypoint_index = waypoint_index + 1;
        if waypoint_index > 5
            break
        end
        begin_point = WayPoints(:, waypoint_index-1);
        end_point= WayPoints(:, waypoint_index);
    end

    plot3([WayPoints(1,:)], [WayPoints(2,:)], [WayPoints(3,:)], 'o--')
    hold on

    unit_velocity = UAV.velocity./norm(UAV.velocity);
    reference_command = straight_line_vector_field(begin_point, end_point, UAV.position, K1, K2);
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
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    frame = getframe(fig1); 
    img = frame2im(frame); 
    [imind, cm] = rgb2ind(img,256); 
    
    if frame_idx == 1
        imwrite(imind,cm,filename,'gif','Loopcount',1,'DelayTime',1/30);                
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',1/30); 
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    time = time + sampling_time;
    pause(0.01);
    frame_idx = frame_idx + 1;
end

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