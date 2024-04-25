
% target position
x = 0.216;
y = 0;
z = 0.026;
target = 0;


tt = target*pi/180;   % target theta in radians 
disp(['tt: ', num2str(tt)]);

% 初始位置
theta_array = inv_kin_robot(x, y, z, tt);  % x y z theta_target

theta0_val = theta_array(1);
theta1_val = theta_array(2);
theta2_val=  theta_array(3);
theta3_val = theta_array(4);

disp(['theta0: ', num2str(theta0_val/4096*360)]);
disp(['theta1: ', num2str(theta1_val/4096*360)]);
disp(['theta2: ', num2str(theta2_val/4096*360)]);
disp(['theta3: ', num2str(theta3_val/4096*360)]);



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
%disp(['theta2!: ', num2str(theta2)]);

% Finding theta1
k1 = 0.13 + 0.124 * cos(theta2);
k2 = 0.124 * sin(theta2);
%5disp(['k1!: ', num2str(k1)]);
%disp(['k2!: ', num2str(k2)]);

theta1 = atan2(Z, X_Y) - atan2(k2, k1);
%disp(['theta1!: ', num2str(theta1)]);

% Finding theta3
theta3 = theta_target - (theta1 + theta2);
%disp(['theta3!: ', num2str(theta3)]);

% Including offset
%angle_offset = acos(0.128 / 0.13);
%angle_offset_new = (angle_offset / (2 * pi)) * 4096;

% Converting simulated theta (in radians) to robot theta and including offsets
%offset = [0, -angle_offset_new, +angle_offset_new, 0];
%offset = [0, 0, 0, 0];
theta0 = 2048 + (theta0/(2*pi))*4096;
theta1 = 3072 - (theta1/(2*pi))*4096;
theta2 = 1024 - (theta2/(2*pi))*4096;
theta3 = 2048 - (theta3/(2*pi))*4096;

theta_array = [theta0, theta1, theta2, theta3];

end
