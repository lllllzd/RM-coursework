% 创建视频写入对象
videoFile = 'task1d.mp4'; % 指定视频文件名
writerObj = VideoWriter(videoFile, 'MPEG-4'); % 创建一个 MPEG-4 文件
writerObj.FrameRate = 20; % 可以设置视频的帧率
open(writerObj);


% DH Parameters
DH_params = [
    0, 0.077, 0, 0;        % Link 1
    0, 0, 0, pi/2;         % Link 2
    0, 0, 0.13, 0;         % Link 3
    0, 0, 0.124, 0;        % Link 4
    0, 0, 0.126, 0         % Link 5
];

% 生成动画帧数
num_frames = 50;
path_x = linspace(-0.2, 0.2, num_frames); % X 轴上从 0.1m 到 0.2m
path_y = linspace(0.1, 0.1, num_frames); % Y
path_z = linspace(0, 0, num_frames); % 
%theta_target = linspace(-pi/2, -pi/2, num_frames); % 旋转角从 0 到 pi/4


% 初始化图形
figure;
axis equal;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
hold on;
view(3);

axis([-0.5 0.5 -0.3 0.1 -0.3 0.3]); % 根据您的实际需要调整这些界限
axis manual;

% 初始化存储图形句柄的数组
h_frames = [];
h_joints = [];
h_frames_vec = [];  % 用于存储关节坐标系的向量图形句柄

% 更新动画帧
for frame = 1:num_frames
    tt = calculatePathAngle(path_x, path_y, frame)
    x = path_x(frame);
    y = path_y(frame);
    z = path_z(frame);
    \

    % 计算逆运动学
    theta = inv_kin_robot(x, y, z, tt);

    % 使用新的关节角度计算正向运动学
    T_total = eye(4);
    points = [0 0 0];  % 初始化点阵列，包括基点

    if ~isempty(h_frames)
        delete(h_frames);  % 删除上一帧的机械臂线图
    end
    if ~isempty(h_joints)
        delete(h_joints);  % 删除上一帧的关节点图
    end
    arrayfun(@delete, h_frames_vec);  % 删除上一帧的所有关节坐标系向量图
    h_frames_vec = [];

    % 绘制原点的坐标系
    h_vec = plotFrame(eye(4));
    h_frames_vec = [h_frames_vec, h_vec];

    for j = 1:size(DH_params, 1)
        T = dhToMatrix(theta(j) + DH_params(j, 1), DH_params(j, 2), DH_params(j, 3), DH_params(j, 4));
        T_total = T_total * T;
        points = [points; T_total(1:3, 4)'];

        % 绘制关节坐标系
        if j > 1
            h_vec = plotFrame(T_total);
            h_frames_vec = [h_frames_vec, h_vec];
        end
    end

    % 更新图像
    h_joints = plot3(points(:,1), points(:,2), points(:,3), 'ko', 'MarkerFaceColor', 'k');
    h_frames = plot3(points(:,1), points(:,2), points(:,3), 'k-', 'LineWidth', 2);
    
    if ~isempty(h_frames_vec)
        arrayfun(@(h) uistack(h, 'top'), h_frames_vec);
    end

    currentFrame = getframe(gcf);
    writeVideo(writerObj, currentFrame);

    % 在每次迭代结束时暂停
    pause(0.01); % 暂停0.1秒以创建动画效果
end

close(writerObj);


%%%%%%%%function%%%%%%%%%


function theta_target = calculatePathAngle(path_x, path_y, frame)
    if frame > 1
        dx = path_x(frame) - path_x(frame - 1);
        dy = path_y(frame) - path_y(frame - 1);
        theta_target = atan2(dy, dx);
    else
        theta_target = -90; % 或其他合适的初始值
    end
end




function vec_handles = plotFrame(T)
    scale = 0.02; % Adjust this scale for the frame arms
    % Draw the frame arms
    h1 = quiver3(T(1,4), T(2,4), T(3,4), scale*T(1,1), scale*T(2,1), scale*T(3,1), 'b', 'LineWidth', 2);
    h2 = quiver3(T(1,4), T(2,4), T(3,4), scale*T(1,2), scale*T(2,2), scale*T(3,2), 'r', 'LineWidth', 2);
    h3 = quiver3(T(1,4), T(2,4), T(3,4), scale*T(1,3), scale*T(2,3),scale*T(3,3), 'g', 'LineWidth', 2);
    vec_handles = [h1, h2, h3];  % 返回创建的向量图形句柄
end

% Function to convert DH parameters to a transformation matrix
function T = dhToMatrix(theta, d, a, alpha)
    T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end

function [theta_array] = inv_kin_robot(x, y, z, theta_target)
   % Determines the angle of the first servo from the base
theta0 = atan2(y, x);

% Determines magnitude of the length of the end-effector in the x-y plane
x_y = sqrt(x^2 + y^2);

% Determines magnitude of the length of the joint preceding the
% end-effector from the origin in the x-y plane
X_Y = x_y - 0.14*cos(theta_target);

% Determines magnitude of the length of the joint preceding the
% end-effector in the vertical plane
Z = z - 0.077 - 0.14*sin(theta_target); % Considering offset from origin

% Finding theta2
c2 = ((X_Y)^2 + (Z)^2 - (0.13)^2 - (0.124)^2) / (2 * 0.13 * 0.124); % negative
s2 = -sqrt(1 - (c2)^2); % The plus or minus in front of the sqrt defines elbow up or down，positive
theta2 = atan2(s2, c2); % negative


% Finding theta1
k1 = 0.13 + 0.124 * cos(theta2);
k2 = 0.124 * sin(theta2);
disp(['k1!: ', num2str(k1)]);
disp(['k2!: ', num2str(k2)]);
disp(['atan2(Z, X_Y): ', num2str(atan2(Z, X_Y))]);
disp(['atan2(k2, k1): ', num2str(atan2(k2, k1))]);

theta1 = atan2(Z, X_Y) - atan2(k2, k1);

% Finding theta3
theta3 = theta_target - (theta1 + theta2);

disp(['theta1!: ', num2str(theta1 * (180 / pi))]);
disp(['theta2!: ', num2str(theta2 * (180 / pi))]);
disp(['theta3!: ', num2str(theta3 * (180 / pi))]);

theta_array = [0,theta0, theta1, theta2, theta3];

end