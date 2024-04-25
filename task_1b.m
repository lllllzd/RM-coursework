
videoFile = 'task1b.mp4'; 
writerObj = VideoWriter(videoFile, 'MPEG-4'); 
writerObj.FrameRate = 20; 
open(writerObj);


% DH Parameters
DH_params = [
    0, 0.077, 0, 0;        % Link 1
    0, 0, 0, pi/2;         % Link 2
    0, 0, 0.13, 0;         % Link 3
    0, 0, 0.124, 0;        % Link 4
    0, 0, 0.126, 0         % Link 5
];


num_frames = 50;


final_theta = [0, -pi/2, 2*pi/3, -pi/2, -pi/2]; 

delta_theta_ranges = zeros(size(DH_params, 1), num_frames);
for i = 1:size(DH_params, 1)
    delta_theta_ranges(i, :) = linspace(0, final_theta(i), num_frames);  
end

initial_theta = [0, 0, pi/2, 0, 0];  



figure;
axis equal;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
hold on;
view(3);

axis([-0.1 0.5 -0.3 0.1 0 0.3]); 
axis manual;



h_frames = [];
h_joints = [];
h_frames_vec = [];  



for frame = 1:num_frames
    theta = initial_theta + delta_theta_ranges(:, frame); 

    
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

 
    
    h_joints = plot3(points(:,1), points(:,2), points(:,3), 'ko', 'MarkerFaceColor', 'k');
    h_frames = plot3(points(:,1), points(:,2), points(:,3), 'k-', 'LineWidth', 2);
    
    if ~isempty(h_frames_vec)
        arrayfun(@(h) uistack(h, 'top'), h_frames_vec);
    end

    currentFrame = getframe(gcf);
    writeVideo(writerObj, currentFrame);

  
    pause(0.01);
end


close(writerObj);

function vec_handles = plotFrame(T)
    scale = 0.06;

    h1 = quiver3(T(1,4), T(2,4), T(3,4), scale*T(1,1), scale*T(2,1), scale*T(3,1), 'b', 'LineWidth', 2);
    h2 = quiver3(T(1,4), T(2,4), T(3,4), scale*T(1,2), scale*T(2,2), scale*T(3,2), 'r', 'LineWidth', 2);
    h3 = quiver3(T(1,4), T(2,4), T(3,4), scale*T(1,3), scale*T(2,3),scale*T(3,3), 'g', 'LineWidth', 2);
    vec_handles = [h1, h2, h3]; 
end

function T = dhToMatrix(theta, d, a, alpha)
    T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end
