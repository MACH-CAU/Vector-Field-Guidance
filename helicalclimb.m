radius = 20;
heights = linspace(0, 5, 6)' * 5;
addpath("lib\");

hexagon_points = generate_polygon_points(radius, 6);
hexagon_points = [hexagon_points, heights];
plot3(hexagon_points(:, 1), hexagon_points(:, 2), hexagon_points(:, 3), 'o--');
%%
% 정팔각형의 점들
octagon_points = generate_polygon_points(radius, 8);
plot(octagon_points(:, 1), octagon_points(:, 2), 'o-')

function points = generate_polygon_points(radius, sides)
    points = zeros(sides, 2);
    for i = 0:sides-1
        angle = (2 * pi / sides) * i;
        x = radius * cos(angle);
        y = radius * sin(angle);
        points(i+1, :) = [x, y];
    end
end
