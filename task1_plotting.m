% DH Parameters
DH_params = [
    0, 0.077, 0, 0;        % Link 1: theta1 = 0, d1 = 0.077, a1 = 0,     alpha1 = 0
    0, 0, 0, pi/2;         % Link 2: theta2 = 0, d2 = 0,     a2 = 0,     alpha2 = pi/2
    0, 0, 0.13, 0;         % Link 3: theta3 = 0, d3 = 0,     a3 = 0.13,  alpha3 = 0
    0, 0, 0.124, 0;        % Link 4: theta4 = 0, d4 = 0,     a4 = 0.124, alpha4 = 0
    0, 0, 0.126, 0         % Link 5: theta5 = 0, d5 = 0,     a5 = 0.126, alpha5 = 0
];

% Example joint angles
%theta = [pi/4, 0, 3*pi/4, -pi/2, -pi/2]; % Replace with actual joint angles

% 假定你的DH参数和dhToMatrix函数已经定义好了

% 定义关节角度的范围，例如从0到2π
theta1_range = linspace(0, 2*pi, 20); % 生成100个线性间隔的点

% 初始化图形
figure;
axis equal;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
hold on;
view(3);

for i = 1:length(theta1_range)
    % 更新第一个关节的角度
    theta(1) = theta1_range(i); % 这里假设你只改变第一个关节的角度
    
    % 使用新的关节角度计算正向运动学
    T_total = eye(4);
    points = [];
    for j = 1:size(DH_params, 1)
        T = dhToMatrix(theta(j) + DH_params(j, 1), DH_params(j, 2), DH_params(j, 3), DH_params(j, 4));
        T_total = T_total * T;
        points = [points; T_total(1:3, 4)'];
    end
    
    % 更新图像
    h = plot3(points(:,1), points(:,2), points(:,3), 'k-', 'LineWidth', 2);
    plot3(points(:,1), points(:,2), points(:,3), 'ko', 'MarkerFaceColor', 'k');
    
    % 在每次迭代结束时暂停
    pause(0.1); % 暂停0.1秒以创建动画效果
    
    % 如果不是最后一次迭代，删除前一个机械臂的图像
    if i < length(theta1_range)
        delete(h);
    end
end


%plotRobot(DH_params, theta);

function plotRobot(DH_params, theta)
    % Initialize the figure
    figure;
    hold on;
    grid on;
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');

    % Initialize transformation
    T_total = eye(4);
    points = []; % Initialize an empty matrix to store points
   

   for i = 1:size(DH_params, 1)
    T = dhToMatrix(theta(i) + DH_params(i, 1), DH_params(i, 2), DH_params(i, 3), DH_params(i, 4));
    T_total = T_total * T;

    % Store the position of each joint
    points = [points; T_total(1:3, 4)'];

    % Plot each joint
    plot3(T_total(1,4), T_total(2,4), T_total(3,4), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');

    if i > 1 % This skips plotting the frame for the first joint
        % Plot frames
        plotFrame(T_total);
    end
end

    % Connect the points with a line
    plot3(points(:,1), points(:,2), points(:,3), 'k-', 'LineWidth', 2);
    % Draw the arm from the origin to the first joint
    plot3([0, 0], [0, 0], [0, DH_params(1, 2)], 'k-', 'LineWidth', 2);
    plotFrame(eye(4));
   
    % Adjust the view
    view(3);
    hold off;
end


% Function to draw a frame
function plotFrame(T)
    scale = 0.02; % Adjust this scale for the frame arms
    % Draw the frame arms
    quiver3(T(1,4), T(2,4), T(3,4), scale*T(1,1), scale*T(2,1), scale*T(3,1), 'r', 'LineWidth', 2);
    quiver3(T(1,4), T(2,4), T(3,4), scale*T(1,2), scale*T(2,2), scale*T(3,2), 'b', 'LineWidth', 2);
    quiver3(T(1,4), T(2,4), T(3,4), scale*T(1,3), scale*T(2,3), scale*T(3,3), 'g', 'LineWidth', 2);
end


% Function to convert DH parameters to a transformation matrix
function T = dhToMatrix(theta, d, a, alpha)
    T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end


