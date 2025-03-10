function success = sendPIDParameters(serialObj, Kp, Ki, Kd)
    success = false;
    
    % Check if serial object is valid
    if ~strcmp(serialObj.Status, 'open')
        warning('Serial port is not open or invalid');
        return;
    end
    
    % Validate matrix dimensions
    if ~isequal(size(Kp), [4, 4]) || ~isequal(size(Ki), [4, 4]) || ~isequal(size(Kd), [4, 4])
        warning('PID matrices must be 4x4');
        return;
    end
    
    % Format matrices for STM32 parsing - convert to flat array with comma separation
    % This creates a simple comma-delimited string of all values in row-major order
    kp_str = matrixToFlatString(Kp);
    ki_str = matrixToFlatString(Ki);
    kd_str = matrixToFlatString(Kd);
    
    % Create message with 'T' prefix and '#' separators between matrices
    pid_msg = ['CC', kp_str, '#', ki_str, '#', kd_str];
    
    try
        % Send data through serial port
        writeline(serialObj, pid_msg);
        disp(['PID parameters sent: ', pid_msg]);
        success = true;
    catch exception
        warning(['Failed to send PID parameters: ', exception.message]);
    end
end



