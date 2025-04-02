function success = sendTrajData(serialObj, C, T)
    success = false;
    
    Check if serial object is valid
    if ~strcmp(serialObj.Status, 'open')
        warning('Serial port is not open or invalid');
        return;
    end
    
    % Validate inputs
    [rows, cols] = size(C);
    if rows ~= 12 || cols ~= 6
        warning('Coefficient matrix C must be 12x6 (2 joints x 6 phases, 6 coefficients per trajectory)');
        return;
    end
    
    if length(T) ~= 5
        warning('Duration vector T must have 6 elements (one for each phase)');
        return;
    end
    
    % Format coefficients and durations for STM32 parsing
    coeff_str = matrixToFlatStringTraj(C);
    dur_str = matrixToFlatStringTraj(T);
    
    % Create message with 'TC' prefix and 'T' separator
    traj_msg = ['TC', coeff_str, 'T', dur_str];
    
    disp(['Trajectory data sent: ', num2str(length(traj_msg)), ' bytes']);
    disp(traj_msg);

    try
        % Send data through serial port
        writeline(serialObj, traj_msg);

        success = true;
    catch exception
        warning(['Failed to send trajectory data: ', exception.message]);
    end

end

% Helper function to convert a matrix to a flat string
function flatStr = matrixToFlatStringTraj(M)
    % Transpose to send data row by row (row-major) order
    M_transposed = M';
    
    % Initialize empty string
    flatStr = '';
    
    % Process each number to optimize size
    for i = 1:numel(M_transposed)
        val = M_transposed(i);
        
        % Check if the value is a round number (integer)
        if val == round(val)
            % For integers, don't include decimal places
            flatStr = [flatStr, sprintf('%d,', val)];
        else
            % For non-integers, use 3 decimal places
            flatStr = [flatStr, sprintf('%.4f,', val)];
        end
    end
    
    % Remove trailing comma
    if ~isempty(flatStr)
        flatStr = flatStr(1:end-1);
    end
end