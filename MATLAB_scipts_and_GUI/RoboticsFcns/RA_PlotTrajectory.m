function RA_PlotTrajectory(robot, q, T, wpts, tpts)

    actual_xyz = zeros(3, size(q,2));
    for i = 1:size(q,2)
        jointAngles = q(:, i)';
        % Compute forward kinematics
        Ti = RA_ForwardKinematics(robot, jointAngles, false);
        actual_xyz(:, i) = Ti(1:3, 4);
    end
    
    % Apply threshold to ignore values below 0.0001
    threshold = 0.0001;
    actual_xyz(abs(actual_xyz) < threshold) = 0;  % Set values below the threshold to 0
    
    figure('Name', 'Trajectory with Waypoints');
    labels = {'X', 'Y', 'Z'};
    
    for dim = 1:3
        subplot(3,1,dim)
        plot(T, actual_xyz(dim,:), 'b-', 'LineWidth', 1.5)
        hold on
        for i = 1:3
            waypointIndex = find(T >= tpts(i), 1);
            plot(T(waypointIndex), wpts(dim,i), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r')
        end
        title([labels{dim} ' Position'])
        ylabel([labels{dim} ' (m)'])
        grid on
        hold off
        if dim == 3
            xlabel('Time (s)')
        end
    end
    legend('Trajectory', 'Waypoints', 'Location', 'best')
end
