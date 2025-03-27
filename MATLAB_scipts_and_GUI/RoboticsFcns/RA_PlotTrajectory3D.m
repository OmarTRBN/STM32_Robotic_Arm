function RA_PlotTrajectory3D(robot, q, wpts)
    actual_xyz = zeros(3, size(q,2));
    
    % Compute forward kinematics for each joint angle set
    for i = 1:size(q,2)
        jointAngles = q(:, i)';
        Ti = RA_ForwardKinematics(robot, jointAngles, false);
        actual_xyz(:, i) = Ti(1:3, 4);
    end
    
    % Apply threshold to remove very small values
    threshold = 0.0001;
    actual_xyz(abs(actual_xyz) < threshold) = 0;

    % Create 3D plot
    figure('Name', '3D Trajectory with Waypoints');
    plot3(actual_xyz(1,:), actual_xyz(2,:), actual_xyz(3,:), 'b-', 'LineWidth', 1.5);
    hold on;
    
    % Mark waypoints in red
    scatter3(wpts(1,:), wpts(2,:), wpts(3,:), 50, 'r', 'filled');

    % Labels and formatting
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('3D Trajectory of Robot End-Effector');
    grid on;
    legend('Trajectory', 'Waypoints', 'Location', 'best');
    axis equal; % Keep aspect ratio balanced
    hold off;
end
