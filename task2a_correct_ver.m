% Read the position of the dynamixel horn with the torque off
% The code executes for a given amount of time then terminates


clc;
clear all;

lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

%% ---- Control Table Addresses ---- %%

ADDR_PRO_TORQUE_ENABLE       = 64;           % Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION       = 116; 
ADDR_PRO_PRESENT_POSITION    = 132; 
ADDR_PRO_OPERATING_MODE      = 11;

%% ---- Other Settings ---- %%

% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
DXL_ID                       = 11;            % Dynamixel ID: 1
DXL_ID1                      = 12;            % Dynamixel ID: 1
DXL_ID2                      = 13;
DXL_ID3                      = 14;
DXL_ID4                      = 15;
BAUDRATE                    = 115200;
DEVICENAME                  = '/dev/tty.usbserial-FT5NUSO1';       % Check which port is being used on your controller
                                            % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
                                            
TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = -150000;      % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 150000;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

ADDR_MAX_POS = 48;
ADDR_MIN_POS = 52;
MAX_POS = 3400;
MIN_POS = 600;


%% ------------------ %%

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

index = 1;
dxl_comm_result = COMM_TX_FAIL;           % Communication result
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];  % Goal position
dxl_error = 0;                              % Dynamixel error
dxl_present_position = 0;                   % Present position


% Open port
if (openPort(port_num))
    fprintf('Port Open\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port\n');
    input('Press any key to terminate...\n');
    return;
end


% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Baudrate Set\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

% target position setup 目标位置计算

target = -90;
tt = target * pi / 180;

% 定义目标位置和目标角度的数组
targets = [
    0.21, 0,  0.16, tt/4;      %1  初始位置  

    0.223, -0.006, 0.03, tt;     %2  移动0 方块1位置 
    0.14, 0, 0.1, tt;      %3 移动1 夹起  
    0.09, 0.0028, 0.02, tt;  %4  移动2 放下位置1  % position2 
    0.17, 0, 0.1, tt;       %5  移动3 向上
    0.1, 0.1,0.1,tt;     %6  方块2上面
    0.145, 0.145, 0.024,tt;      %7 方块2 夹
    0.04, 0.08,0.12,tt;       %8 方块2上去
    -0.003, 0.088, 0.02,tt;      %9 方块2放下位置   % position3 

    0.05, 0.1, 0.1, tt;         %10 初始位置  
    0.04, -0.15, 0.08, tt;    %11  方块3上面
    0.07, -0.198, 0.027, tt;  %12  方块3位置 
    0.1, -0.14, 0.1,tt;   %13  
    0.12, -0.12, 0.02,tt;    %14 放下位置3  % position1
    0.06, -0.1, 0.07, tt;       %15   
    0.1, 0, 0.1, tt/2;       %16  初始位置 
    0.13, 0, 0.12, tt;   %17
    
];

% 预分配存储结果的数组
theta_vals = zeros(size(targets, 1), 4); % 每行存储一个移动的结果

% 循环处理每一个移动
for i = 1:size(targets, 1)
    % 调用逆运动学函数
    move_result = inv_kin_robot(targets(i, 1), targets(i, 2), targets(i, 3), targets(i, 4));    
    % 存储结果
    theta_vals(i, :) = move_result;
end



% Put actuator into Position Control Mode
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID,  ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_OPERATING_MODE, 3);

% ----- SET MOTION LIMITS ----------- %
ADDR_MAX_POS = 48;
ADDR_MIN_POS = 52;
MAX_POS = 3400;
MIN_POS = 600;

% Set max position limit
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_MAX_POS, MAX_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_MAX_POS, MAX_POS);
% Set min position limit
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_MIN_POS, MIN_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_MIN_POS, MIN_POS);

% enable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID , ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_TORQUE_ENABLE, 1);


% velocity
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID , 112, 700);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, 112, 700);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, 112, 700);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, 112, 700);

% initial position  最开始待机的位置
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID , 116, theta_vals(1, 1));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, 116, theta_vals(1, 2));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, 116, theta_vals(1, 3));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, 116, theta_vals(1, 4));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, 116, 2843); 
pause(0.8); %待机暂停

%移动到第一个方块
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID , 116, theta_vals(2, 1));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, 116, theta_vals(2, 2));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, 116, theta_vals(2, 3));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, 116, theta_vals(2, 4));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, 116, 2843); % gripper open

pause(0.5);

%夹起
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, 116, 1580); % gripper close

pause(0.5);

%夹起向上
% move 1
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID , 116, theta_vals(3, 1));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, 116, theta_vals(3, 2));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, 116, theta_vals(3, 3));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, 116, theta_vals(3, 4));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, 116, 1600); % gripper close
pause(0.5);

%移动到第一个位置放下
% move 2
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID , 116, theta_vals(4, 1));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, 116, theta_vals(4, 2));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, 116, theta_vals(4, 3));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, 116, theta_vals(4, 4));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, 116, 1600); % gripper close
pause(1);
%放开方块
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, 116, 2843); % gripper open

pause(0.5);

% move 3 向上移动一下
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID , 116, theta_vals(5, 1));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, 116, theta_vals(5, 2));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, 116, theta_vals(5, 3));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, 116, theta_vals(5, 4));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, 116, 2843);
pause(0.5);

% move 4 到第二个方块的位置
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID , 116, theta_vals(6, 1));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, 116, theta_vals(6, 2));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, 116, theta_vals(6, 3));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, 116, theta_vals(6, 4));
pause(0.5);

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID , 116, theta_vals(7, 1));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, 116, theta_vals(7, 2));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, 116, theta_vals(7, 3));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, 116, theta_vals(7, 4));
pause(0.5);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, 116, 1580); % gripper close
pause(0.5);

%夹起
% move 
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID , 116, theta_vals(8, 1));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, 116, theta_vals(8, 2));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, 116, theta_vals(8, 3));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, 116, theta_vals(8, 4));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, 116, 1580); % gripper close
pause(0.5);

% move 3 放下的位置
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID , 116, theta_vals(9, 1));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, 116, theta_vals(9, 2));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, 116, theta_vals(9, 3));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, 116, theta_vals(9, 4));
pause(1);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, 116, 2843);
pause(0.5);

% move 4 
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID , 116, theta_vals(10, 1));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, 116, theta_vals(10, 2));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, 116, theta_vals(10, 3));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, 116, theta_vals(10, 4));
pause(0.5);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID , 116, theta_vals(17, 1));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, 116, theta_vals(17, 2));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, 116, theta_vals(17, 3));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, 116, theta_vals(17, 4));
pause(0.5);


% move 
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID , 116, theta_vals(11, 1));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, 116, theta_vals(11, 2));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, 116, theta_vals(11, 3));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, 116, theta_vals(11, 4));
pause(0.5);

%移动到第二个位置放下
% move 
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID , 116, theta_vals(12, 1));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, 116, theta_vals(12, 2));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, 116, theta_vals(12, 3));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, 116, theta_vals(12, 4));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, 116, 2843); % gripper open
pause(0.5);

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, 116, 1580); % gripper close
pause(0.5);


% move  向上移动一下
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID , 116, theta_vals(13, 1));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, 116, theta_vals(13, 2));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, 116, theta_vals(13, 3));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, 116, theta_vals(13, 4));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, 116, 1580); % gripper close
pause(0.5);


write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID , 116, theta_vals(14, 1));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, 116, theta_vals(14, 2));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, 116, theta_vals(14, 3));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, 116, theta_vals(14, 4));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, 116, 1600);  % gripper close
pause(1.2);

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, 116, 2843);  % gripper open
pause(0.5);

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID , 116, theta_vals(15, 1));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, 116, theta_vals(15, 2));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, 116, theta_vals(15, 3));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, 116, theta_vals(15, 4));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, 116, 2843); % gripper open
pause(0.5);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID , 116, theta_vals(16, 1));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, 116, theta_vals(16, 2));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, 116, theta_vals(16, 3));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, 116, theta_vals(16, 4));
pause(0.5);



dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);

if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end

%{
i = 0;
    j = 0;
    while (j<200)
        j = j+1;
        
        % Read present position
        dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_POSITION);
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        
        dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
        
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
        end

        fprintf('[ID:%03d] Position: %03d\n', DXL_ID, typecast(uint32(dxl_present_position), 'int32'));

        if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl_present_position), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
            break;
        end
    end
%}


% Disable Dynamixel Torque
%{
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
%}


dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end



% Close port
closePort(port_num);
fprintf('Port Closed \n');

% Unload Library
unloadlibrary(lib_name);
close all;
clear all;


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

% Including offset
angle_offset = acos(0.128 / 0.13);
angle_offset_new = (angle_offset / (2 * pi)) * 4096;

% Converting simulated theta (in radians) to robot theta and including offsets
offset = [40, -angle_offset_new, +angle_offset_new, 0];

theta0 = 2048 + (theta0/(2*pi))*4096+ offset(1);
theta1 = 3072 - (theta1/(2*pi))*4096+ offset(2);
theta2 = 1024 - (theta2/(2*pi))*4096+ offset(3);
theta3 = 2048 - (theta3/(2*pi))*4096+ offset(4);

theta_array = [theta0, theta1, theta2, theta3];

end
