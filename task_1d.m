
videoFile = 'task1d.mp4';
writerObj = VideoWriter(videoFile, 'MPEG-4'); 
writerObj.FrameRate = 20; 
open(writerObj);


num_frames = 120;


DH_params = [
    0, 0.077, 0, 0;        % Link 1
    0, 0, 0, pi/2;         % Link 2
    0, 0, 0.13, 0;         % Link 3
    0, 0, 0.124, 0;        % Link 4
    0, 0, 0.126, 0         % Link 5
];


side_length = 0.1;  
num_points_per_side=10;


p1 = [0.2, 0.-0.05, 0.07]; 
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
path_z = ones(1, 40)*0.07;  
num_frames_per_square = 40;




xz_p1 = [0.2, 0, 0.02];
xz_p2 = [0.1, 0, 0.02];
xz_p3 = [0.1, 0, 0.12];
xz_p4 = [0.2, 0, 0.12];


xz_path_x = [linspace(xz_p1(1), xz_p2(1), num_points_per_side), ...
             linspace(xz_p2(1), xz_p3(1), num_points_per_side), ...
             linspace(xz_p3(1), xz_p4(1), num_points_per_side), ...
             linspace(xz_p4(1), xz_p1(1), num_points_per_side)];
xz_path_z = [linspace(xz_p1(3), xz_p2(3), num_points_per_side), ...
             linspace(xz_p2(3), xz_p3(3), num_points_per_side), ...
             linspace(xz_p3(3), xz_p4(3), num_points_per_side), ...
             linspace(xz_p4(3), xz_p1(3), num_points_per_side)];
xz_path_y = zeros(1, num_frames_per_square); 


yz_p1 = [0.15, -0.05, 0.02];
yz_p2 = [0.15, 0.05, 0.02];
yz_p3 = [0.15, 0.05, 0.12];
yz_p4 = [0.15, -0.05, 0.12];


yz_path_y = [linspace(yz_p1(2), yz_p2(2), num_points_per_side), ...
             linspace(yz_p2(2), yz_p3(2), num_points_per_side), ...
             linspace(yz_p3(2), yz_p4(2), num_points_per_side), ...
             linspace(yz_p4(2), yz_p1(2), num_points_per_side)];
yz_path_z = [linspace(yz_p1(3), yz_p2(3), num_points_per_side), ...
             linspace(yz_p2(3), yz_p3(3), num_points_per_side), ...
             linspace(yz_p3(3), yz_p4(3), num_points_per_side), ...
             linspace(yz_p4(3), yz_p1(3), num_points_per_side)];
yz_path_x = ones(1, num_frames_per_square)*0.15; 


total_path_x = [path_x, xz_path_x, yz_path_x];
total_path_y = [path_y, xz_path_y, yz_path_y];
total_path_z = [path_z, xz_path_z, yz_path_z];


theta_xy = linspace(-pi/2, -pi/2, num_frames_per_square);


theta_xz = linspace(-pi/4, -pi/4, num_frames_per_square);


theta_yz = linspace(-pi/3, -pi/3, num_frames_per_square);

total_theta_target = [theta_xy, theta_xz, theta_yz];



num_frames = length(total_path_x); 




figure;
axis equal;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
hold on;
view(3);

axis([-0.2 0.3 -0.3 0.3 0 0.3]); 
axis manual;


path_points = [];
h_frames = [];
h_joints = [];
h_frames_vec = [];  
colors = ['r', 'g', 'b']; 


for plane = 1:3 
    path_points = [];  
    current_color = colors(plane);

    for frame = 1:40
       
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

       
        theta = inv_kin_robot(x, y, z, tt);

   
        T_total = eye(4);
        points = [0 0 0];  


        if ~isempty(h_frames)
            delete(h_frames); 
        end
        if ~isempty(h_joints)
            delete(h_joints);  
        end
        arrayfun(@delete, h_frames_vec);  
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
       
        plot3(path_points(:,1), path_points(:,2), path_points(:,3), current_color, 'LineWidth', 2);

    
        h_joints = plot3(points(:,1), points(:,2), points(:,3), 'ko', 'MarkerFaceColor', 'k');
        h_frames = plot3(points(:,1), points(:,2), points(:,3), 'k-', 'LineWidth', 2);
        
        if ~isempty(h_frames_vec)
            arrayfun(@(h) uistack(h, 'top'), h_frames_vec);
        end
        currentFrame = getframe(gcf);
        writeVideo(writerObj, currentFrame);

     
        pause(0.001); 
    end
end

close(writerObj);

function vec_handles = plotFrame(T)
    scale = 0.04; % Adjust this scale for the frame arms
    % Draw the frame arms
    h1 = quiver3(T(1,4), T(2,4), T(3,4), scale*T(1,1), scale*T(2,1), scale*T(3,1), 'b', 'LineWidth', 2);
    h2 = quiver3(T(1,4), T(2,4), T(3,4), scale*T(1,2), scale*T(2,2), scale*T(3,2), 'r', 'LineWidth', 2);
    h3 = quiver3(T(1,4), T(2,4), T(3,4), scale*T(1,3), scale*T(2,3),scale*T(3,3), 'g', 'LineWidth', 2);
    vec_handles = [h1, h2, h3];  
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
s2 = -sqrt(1 - (c2)^2); 
theta2 = atan2(s2, c2); % negative


% Finding theta1
k1 = 0.13 + 0.124 * cos(theta2);
k2 = 0.124 * sin(theta2);

theta1 = atan2(Y, X) - atan2(k2, k1);

% Finding theta3
theta3 = theta_target - (theta1 + theta2);

theta_array = [0,theta0, theta1, theta2, theta3];

end