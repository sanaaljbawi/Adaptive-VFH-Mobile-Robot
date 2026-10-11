robot_pos = [0.0, 4.0];
theta = deg2rad(45.0);
goal_pos = [6.0,4.0];

x_wall1 = 1.0:0.05:5.5;
y_wall1 = ones(size(x_wall1)) * 7.0;
obstacles = [obstacles; [x_wall1', y_wall1']];

% ÌÏÇÑ ãÇÆá ÓÝáí Øæíá íÕäÚ ããÑ ÖíÞ (íãÊÏ ãä ÇáäÞØÉ (1, 1) Åáì ÇáäÞØÉ (5, 2))
x_wall2 = 1.0:0.05:5.0;
y_wall2 = linspace(1.0, 2.0, length(x_wall2));
obstacles = [obstacles; [x_wall2', y_wall2']];

% ÚÇÆÞ ßÊáí ÖÎã Úáæí ÞÈá ÇáÝÎ (ßÊáÉ ãÑÈÚÉ ããÊáÆÉ ÈÇáäÞÇØ)
[x_blk1, y_blk1] = meshgrid(0.8:0.1:1.3, 5.8:0.1:6.3);
obstacles = [obstacles; [x_blk1(:), y_blk1(:)]];

% ÚÇÆÞ ßÊáí ÖÎã ÓÝáí ÈÚÏ ÇáÝÎ íÛáÞ ÇáãÎÑÌ ÇáãÈÇÔÑ
[x_blk2, y_blk2] = meshgrid(4.5:0.1:5.0, 3.2:0.1:3.8);
obstacles = [obstacles; [x_blk2(:), y_blk2(:)]];

extra_static_obstacles = [
    0.8, 2.8;  1.2, 5.2;  1.8, 1.8;  1.5, 6.2;  % ÚÞÈÇÊ ÞÈá ÏÎæá ÇáÝÎ
    2.5, 4.0;  2.8, 3.5;  2.2, 4.8;             % ÚÞÈÇÊ ãÈÚËÑÉ ÏÇÎá ãäØÞÉ ÇáãäÇæÑÉ
    4.2, 5.0;  4.5, 2.5;  4.8, 5.8;  5.2, 2.0;  % ÚÞÈÇÊ ÈÚÏ ÇáÎÑæÌ ãä ÝæåÉ ÇáÝÎ
    5.5, 4.5;  5.0, 3.2;  3.8, 6.5;  3.8, 1.2   % ÚÞÈÇÊ ÞÑíÈÉ ãä ãÓÇÑ ÇáåÏÝ æÇáäåÇíÉ
];
obstacles = [obstacles; extra_static_obstacles];

W = 0.3;
R = 0.1;
V_max = 0.4;
dt = 0.1;
distance_threshold = 0.3;
max_steps = 1000;

vfh =  VFHPlusController();
vfh.robot_radius = 0.2;
vfh.safety_dist = 0.15;

figure('Name', 'VFH+ Optimization Results', 'NumberTitle', 'off');
hold on;
grid on;
axis([-1 8 -1 8]);
xlabel('X Position (m)');
ylabel('Y Position (m)');

plot(goal_pos(1), goal_pos(2), 'gp', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
plot(obstacles(:, 1), obstacles(:, 2), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');

h_robot = plot(robot_pos(1), robot_pos(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
h_dir = plot([robot_pos(1), robot_pos(1) + 0.4*cos(theta)], [robot_pos(2), robot_pos(2) + 0.4*sin(theta)], 'b-', 'LineWidth', 2);
h_path = plot(robot_pos(1), robot_pos(2), 'k:', 'LineWidth', 1.5);

path_x = robot_pos(1);
path_y = robot_pos(2);
all_steer_angles = [];
total_steps = 0;
reached_destination = false;

while norm(goal_pos - robot_pos) > distance_threshold && total_steps < max_steps
    if ~ishghandle(h_robot)
        break;
    end
    
    target_angle = rad2deg(atan2(goal_pos(2) - robot_pos(2), goal_pos(1) - robot_pos(1)));
    current_angle = rad2deg(theta);
    
    num_obstacles = size(obstacles, 1);
    ranges = zeros(1, num_obstacles);
    angles = zeros(1, num_obstacles);
    
    for i = 1:num_obstacles
        dx = obstacles(i, 1) - robot_pos(1);
        dy = obstacles(i, 2) - robot_pos(2);
        ranges(i) = sqrt(dx^2 + dy^2);
        angles(i) = rad2deg(atan2(dy, dx));
    end
    
    steering_deg = vfh.compute_steering(ranges, angles, target_angle, current_angle);
    all_steer_angles = [all_steer_angles, steering_deg];
    total_steps = total_steps + 1;
    
    steering_rad = deg2rad(steering_deg);
    angle_error = atan2(sin(steering_rad - theta), cos(steering_rad - theta));
    
    V_linear = V_max * cos(angle_error);
    if V_linear < 0
        V_linear = 0.05;
    end
    
    W_angular = 2.5 * angle_error;
    V_r = V_linear + (W_angular * W) / 2;
    V_l = V_linear - (W_angular * W) / 2;
    
    w_r = V_r / R;
    w_l = V_l / R;
    
    V_actual = R * (w_r + w_l) / 2;
    W_actual = R * (w_r - w_l) / W;
    
    robot_pos(1) = robot_pos(1) + V_actual * cos(theta) * dt;
    robot_pos(2) = robot_pos(2) + V_actual * sin(theta) * dt;
    theta = theta + W_actual * dt;
    theta = atan2(sin(theta), cos(theta));
    
    path_x = [path_x, robot_pos(1)];
    path_y = [path_y, robot_pos(2)];
    
    set(h_robot, 'XData', robot_pos(1), 'YData', robot_pos(2));
    set(h_dir, 'XData', [robot_pos(1), robot_pos(1) + 0.4*cos(theta)], 'YData', [robot_pos(2), robot_pos(2) + 0.4*sin(theta)]);
    set(h_path, 'XData', path_x, 'YData', path_y);
    
    drawnow;
    pause(dt);
end

if norm(goal_pos - robot_pos) <= distance_threshold
    reached_destination = true;
end

path_length = sum(sqrt(diff(path_x).^2 + diff(path_y).^2));
oscillation_metric = mean(abs(diff(all_steer_angles)));
navigation_time = total_steps * dt;

fprintf('\n====================================\n');
fprintf('   EXPERIMENTAL NAVIGATION RESULTS   \n');
fprintf('====================================\n');
if reached_destination
    fprintf('Mission Status         : SUCCESS (Goal Reached)\n');
else
    fprintf('Mission Status         : FAILED (Timeout / Trapped)\n');
end
fprintf('Total Navigation Time  : %.2f seconds\n', navigation_time);
fprintf('Total Simulation Steps : %d steps\n', total_steps);
fprintf('Total Path Length      : %.3f meters\n', path_length);
fprintf('Average Oscillation Rate: %.3f deg/step\n', oscillation_metric);
fprintf('====================================\n');