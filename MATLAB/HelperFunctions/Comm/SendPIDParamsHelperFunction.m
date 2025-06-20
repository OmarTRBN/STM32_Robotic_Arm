function SendPIDParamsHelperFunction(app, event)
    source = event.Source;  % Which button triggered it?

    if source == app.SendKpParametersButton
        params = app.PID_KpTable.Data;
        prefix = 'KP';  % Define prefix for Kp parameters
    elseif source == app.SendKiParametersButton
        params = app.PID_KiTable.Data;
        prefix = 'KI';
    elseif source == app.SendKdParametersButton
        params = app.PID_KdTable.Data;
        prefix = 'KD';
    else
        return;  % unknown source
    end

    sendControllerParameters(app.serialObj, params, prefix, app.numJoints);
end

function sendControllerParameters(serialObj, params, prefix, numJoints)
    % Check if serial object is valid
    if ~strcmp(serialObj.Status, 'open')
        warning('Serial port is not open or invalid');
        return;
    end

    % Validate bounds
    if numJoints < 1 || numJoints > 4
        warning('Invalid joint count: %d. Must be 1–4.', numJoints);
        return;
    end

    % Validate matrix dimensions
    if size(params,1) < numJoints || size(params,2) < numJoints
        warning('PID table size is smaller than expected %dx%d block.', numJoints, numJoints);
        return;
    end

    % Extract top-left nxn block
    trimmedParams = params(1:numJoints, 1:numJoints);
    
    % Format for STM32 parsing
    param_str = matrixToFlatString(trimmedParams);
    pid_msg = ['CP', prefix, param_str, newline];
    
    try
        write(serialObj, pid_msg, 'char');
        disp(['PID parameters sent: ', pid_msg]);
    catch exception
        warning(['Failed to send PID parameters: ', exception.message]);
    end
end

function flatStr = matrixToFlatString(matrix)
    % Convert a matrix to a flat comma-delimited string without brackets
    % Example: [1,2;3,4] becomes "1,2,3,4"
    
    % Transpose and reshape to ensure row-major order (easier for STM32 parsing)
    flatMatrix = reshape(matrix', 1, []);
    
    % Convert to string with commas
    flatStr = sprintf('%.1f,', flatMatrix);
    
    % Remove trailing comma
    flatStr = flatStr(1:end-1);
end