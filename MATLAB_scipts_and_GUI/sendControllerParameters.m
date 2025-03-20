function sendControllerParameters(serialObj, params, prefix)
    % Check if serial object is valid
    if ~strcmp(serialObj.Status, 'open')
        warning('Serial port is not open or invalid');
        return;
    end
    
    % Validate matrix dimensions
    if ~isequal(size(params), [4, 4]) 
        warning('Parameters matrix must be 4x4');
        return;
    end
    
    % Format matrices for STM32 parsing
    % Convert to flat array with comma separation
    param_str = matrixToFlatString(params);
    pid_msg = [prefix, param_str];
    
    try
        % Send data through serial port
        writeline(serialObj, pid_msg);
        disp(['PID parameters sent: ', pid_msg]);
    catch exception
        warning(['Failed to send PID parameters: ', exception.message]);
    end
end



