close all
clear all
addpath("lib\");

x_axis = linspace(-10, 10, 11);
y_axis = linspace(-10, 10, 11);

fig1 = figure(1);
hold on
start_point = [-20, -30, 0];
end_point = [20, 30, 0];
chi_q = atan2(end_point(2)-start_point(2), end_point(1)-start_point(1));
traj_unit_vec = (end_point-start_point)./norm(end_point-start_point);

for xdot = x_axis
    for ydot = y_axis
        chi_c = find_vector_field(chi_q, start_point, xdot, ydot);
        dir = [xdot + cos(chi_c), ydot + sin(chi_c)];
        plot([xdot, dir(1)], [ydot, dir(2)], 'b-')
        scatter(dir(1), dir(2), 'b.')
    end
end

plot([start_point(1), end_point(1)], [start_point(2), end_point(2)], 'r-', 'LineWidth', 1)
xlim([-11, 11])
ylim([-11, 11])

function chi_c = find_vector_field(chi_q, start, xdot, ydot)
    dot_new = [xdot, ydot, 0];
    dir = dot_new - start;
    CRSS = cross([cos(chi_q), sin(chi_q), 0], dir);
    CRC = cross([cos(chi_q), sin(chi_q), 0], CRSS)./norm(cross([cos(chi_q), sin(chi_q), 0], CRSS));
    e = abs(dot(CRC, dir));
    if (CRSS(3) <= 0)
        e = -e;
    elseif isnan(CRSS)
        e = 0;
    end
    chi_infty = pi/12;
    k_epy = 6;
    chi_c = chi_q - chi_infty*2/pi*atan(k_epy*e);
end