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
    points = [0, 0, 0]; % Start with the origin

    % Draw the arm from the origin to the first joint
    plot3([0, 0], [0, 0], [0, DH_params(1, 2)], 'k-', 'LineWidth', 2);

    % Plot the robotic arm
    for i = 1:size(DH_params, 1)
        T = dhToMatrix(theta(i) + DH_params(i, 1), DH_params(i, 2), DH_params(i, 3), DH_params(i, 4));
        T_total = T_total * T;

        % Store the position of each joint
        points = [points; T_total(1:3, 4)'];

        % Plot each joint
        plot3(T_total(1,4), T_total(2,4), T_total(3,4), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');

        % Plot frames
        plotFrame(T_total);
    end

    % Connect the points with a line
    plot3(points(:,1), points(:,2), points(:,3), 'k-', 'LineWidth', 2);

    % Adjust the view
    view(3);
    hold off;
end

% Function to draw a frame
function plotFrame(T)
    scale = 0.02; % Adjust this scale for the frame arms
    % Draw the frame arms
    quiver3(T(1,4), T(2,4), T(3,4), scale*T(1,1), scale*T(2,1), scale*T(3,1), 'r', 'LineWidth', 2);
    quiver3(T(1,4), T(2,4), T(3,4), scale*T(1,2), scale*T(2,2), scale*T(3,2), 'g', 'LineWidth', 2);
    quiver3(T(1,4), T(2,4), T(3,4), scale*T(1,3), scale*T(2,3), scale*T(3,3), 'b', 'LineWidth', 2);
end

% Function to convert DH parameters to a transformation matrix
function T = dhToMatrix(theta, d, a, alpha)
    T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end
