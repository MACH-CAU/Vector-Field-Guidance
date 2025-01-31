clc; close all; clear all;
addpath("lib\");


start_point = [10; 10; 10];
end_point = [10; 10; 0];

x_axis = linspace(0, 20, 11);
y_axis = linspace(0, 20, 11);
z_axis = linspace(0, 15, 8);

K1 = [5, 1, 1;...
      1, 5, 1;...
      1, 1, 5];
K2 = [10, 1, 1;...
      1, 10, 1;...
      1, 1, 10];

fig1 = figure(1);
ref_line = plot3([start_point(1), end_point(1)], [start_point(2), end_point(2)], [start_point(3), end_point(3)], '--', 'LineWidth', 3);
hold on
grid on
axis equal

for xdot = x_axis
    for ydot = y_axis
        for zdot = z_axis
            r_pos = [xdot; ydot; zdot];
            unit_vec = -K1*([r_pos(1)-10;0;0] + [0;r_pos(2)-10;0]) + K2*[0; 0; -1];
            unit_vec = unit_vec./norm(unit_vec);
            input_c = r_pos + unit_vec;
            plot3([r_pos(1), input_c(1)], [r_pos(2), input_c(2)], [r_pos(3), input_c(3)], 'r-');
        end
    end
end
xlabel('E-Axis')
ylabel('N-Axis')
zlabel('U-Axis')
title('Vector Field 3D')
