% 创建视频写入对象
videoFile = 'task1d.mp4'; % 指定视频文件名
writerObj = VideoWriter(videoFile, 'MPEG-4'); % 创建一个 MPEG-4 文件
writerObj.FrameRate = 20; % 可以设置视频的帧率
open(writerObj);

% 生成动画帧数
num_frames = 120;

% DH Parameters
DH_params = [
    0, 0.077, 0, 0;        % Link 1
    0, 0, 0, pi/2;         % Link 2
    0, 0, 0.13, 0;         % Link 3
    0, 0, 0.124, 0;        % Link 4
    0, 0, 0.126, 0         % Link 5
];

% 更新正方形路径的定义
side_length = 0.1;  % 正方形边长
num_points_per_side=10;

% 定义正方形的四个顶点
% xy square
p1 = [0.2, 0.-0.05, 0.07];  % 起始点
p2 = [0.1, -0.05, 0.07];
p3 = [0.1, 0.05, 0.07];
p4 = [0.2, 0.05, 0.07];

path_x = [linspace(p1(1), p2(1), num_points_per_side), ...
          linspace(p2(1), p3(1), num_points_per_side), ...
          linspace(p3(1), p4(1), num_points_per_side), ...
          linspace(p4(1), p1(1), num_points_per_side)];
path_y = [linspace(p1(2), p2(2), num_points_per_side), ...
          linspace(p2(2), p3(2), num_points_per_side), ...
          linspace(p3(2), p4(2), num_points_per_side), ...
          linspace(p4(2), p1(2), num_points_per_side)];
path_z = ones(1, 40)*0.07;  % Z坐标保持不变
num_frames_per_square = 40;

% 假设原有XY平面的点数
  % 原来定义的帧数

% 定义XZ平面的四个顶点
xz_p1 = [0.2, 0, 0.02];
xz_p2 = [0.1, 0, 0.02];
xz_p3 = [0.1, 0, 0.12];
xz_p4 = [0.2, 0, 0.12];

% 计算XZ平面的路径点
xz_path_x = [linspace(xz_p1(1), xz_p2(1), num_points_per_side), ...
             linspace(xz_p2(1), xz_p3(1), num_points_per_side), ...
             linspace(xz_p3(1), xz_p4(1), num_points_per_side), ...
             linspace(xz_p4(1), xz_p1(1), num_points_per_side)];
xz_path_z = [linspace(xz_p1(3), xz_p2(3), num_points_per_side), ...
             linspace(xz_p2(3), xz_p3(3), num_points_per_side), ...
             linspace(xz_p3(3), xz_p4(3), num_points_per_side), ...
             linspace(xz_p4(3), xz_p1(3), num_points_per_side)];
xz_path_y = zeros(1, num_frames_per_square); % Y坐标保持不变

% 定义YZ平面的四个顶点
yz_p1 = [0.15, -0.05, 0.02];
yz_p2 = [0.15, 0.05, 0.02];
yz_p3 = [0.15, 0.05, 0.12];
yz_p4 = [0.15, -0.05, 0.12];

% 计算YZ平面的路径点
yz_path_y = [linspace(yz_p1(2), yz_p2(2), num_points_per_side), ...
             linspace(yz_p2(2), yz_p3(2), num_points_per_side), ...
             linspace(yz_p3(2), yz_p4(2), num_points_per_side), ...
             linspace(yz_p4(2), yz_p1(2), num_points_per_side)];
yz_path_z = [linspace(yz_p1(3), yz_p2(3), num_points_per_side), ...
             linspace(yz_p2(3), yz_p3(3), num_points_per_side), ...
             linspace(yz_p3(3), yz_p4(3), num_points_per_side), ...
             linspace(yz_p4(3), yz_p1(3), num_points_per_side)];
yz_path_x = ones(1, num_frames_per_square)*0.15; % X坐标保持不变

% 合并所有平面的路径点
total_path_x = [path_x, xz_path_x, yz_path_x];
total_path_y = [path_y, xz_path_y, yz_path_y];
total_path_z = [path_z, xz_path_z, yz_path_z];

% 原有XY平面的theta_target
theta_xy = linspace(-pi/2, -pi/2, num_frames_per_square);

% XZ平面的theta_target，例如，旋转角为0
theta_xz = linspace(-pi/4, -pi/4, num_frames_per_square);

% YZ平面的theta_target，例如，旋转角为pi/4
theta_yz = linspace(-pi/3, -pi/3, num_frames_per_square);

% 合并所有theta_target
total_theta_target = [theta_xy, theta_xz, theta_yz];


% 更新帧数
num_frames = length(total_path_x);  % 更新总帧数以包括所有路径点



% 初始化图形
figure;
axis equal;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
hold on;
view(3);

axis([-0.2 0.3 -0.3 0.3 0 0.3]); % 根据您的实际需要调整这些界限
axis manual;

% 初始化存储图形句柄的数组
path_points = [];
h_frames = [];
h_joints = [];
h_frames_vec = [];  % 用于存储关节坐标系的向量图形句柄
colors = ['r', 'g', 'b']; 

% 更新动画帧
for plane = 1:3  % 分别处理三个平面
    path_points = [];  % 重新初始化路径点数组
    current_color = colors(plane);

    for frame = 1:40
        % 根据当前平面选择路径和theta_target
        if plane == 1  % XY plane
            x = path_x(frame);
            y = path_y(frame);
            z = path_z(frame);
            tt = theta_xy(frame);
        elseif plane == 2  % XZ plane
            x = xz_path_x(frame);
            y = xz_path_y(frame);
            z = xz_path_z(frame);
            tt = theta_xz(frame);
        else  % YZ plane
            x = yz_path_x(frame);
            y = yz_path_y(frame);
            z = yz_path_z(frame);
            tt = theta_yz(frame);
        end

        % 计算逆运动学
        theta = inv_kin_robot(x, y, z, tt);

        % 使用新的关节角度计算正向运动学
        T_total = eye(4);
        points = [0 0 0];  % 初始化点阵列，包括基点

         % 清除前一帧的图形句柄
        if ~isempty(h_frames)
            delete(h_frames);  % 删除上一帧的机械臂线图
        end
        if ~isempty(h_joints)
            delete(h_joints);  % 删除上一帧的关节点图
        end
        arrayfun(@delete, h_frames_vec);  % 删除上一帧的所有关节坐标系向量图
        h_frames_vec = [];

        h_vec = plotFrame(eye(4));
        h_frames_vec = [h_frames_vec, h_vec];

        for j = 1:size(DH_params, 1)
            T = dhToMatrix(theta(j) + DH_params(j, 1), DH_params(j, 2), DH_params(j, 3), DH_params(j, 4));
            T_total = T_total * T;
            points = [points; T_total(1:3, 4)'];
            if j > 1
             h_vec = plotFrame(T_total);
             h_frames_vec = [h_frames_vec, h_vec];
            end
        end

        path_points = [path_points; points(end, :)];

        % 绘制路径点
        plot3(path_points(:,1), path_points(:,2), path_points(:,3), current_color, 'LineWidth', 2);

        % 更新图像
        h_joints = plot3(points(:,1), points(:,2), points(:,3), 'ko', 'MarkerFaceColor', 'k');
        h_frames = plot3(points(:,1), points(:,2), points(:,3), 'k-', 'LineWidth', 2);
        
        if ~isempty(h_frames_vec)
            arrayfun(@(h) uistack(h, 'top'), h_frames_vec);
        end
        currentFrame = getframe(gcf);
        writeVideo(writerObj, currentFrame);

        % 在每次迭代结束时暂停
        pause(0.001); % 暂停0.1秒以创建动画效果
    end
end

close(writerObj);

function vec_handles = plotFrame(T)
    scale = 0.04; % Adjust this scale for the frame arms
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
X = x_y - 0.126*cos(theta_target);

% Determines magnitude of the length of the joint preceding the
% end-effector in the vertical plane
Y = z - 0.077 - 0.126*sin(theta_target); % Considering offset from origin

% Finding theta2
c2 = ((Y)^2 + (X)^2 - (0.13)^2 - (0.124)^2) / (2 * 0.13 * 0.124); % negative
s2 = -sqrt(1 - (c2)^2); % The plus or minus in front of the sqrt defines elbow up or down，positive
theta2 = atan2(s2, c2); % negative


% Finding theta1
k1 = 0.13 + 0.124 * cos(theta2);
k2 = 0.124 * sin(theta2);

theta1 = atan2(Y, X) - atan2(k2, k1);

% Finding theta3
theta3 = theta_target - (theta1 + theta2);

theta_array = [0,theta0, theta1, theta2, theta3];

end