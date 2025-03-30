function success = sendSetpoint(serialObj, setpoint)
    % Sends setpoint value to an STM32 through serial connection
    %
    % Inputs:
    %   serialObj - Serial port object
    %   setpoint - Target setpoint value
    %
    % Outputs:
    %   success - Boolean indicating whether operation was successful
    
    % Input validation
    success = false;
    
    % Check if serial object is valid
    if ~isvalid(serialObj) || ~strcmp(serialObj.Status, 'open')
        warning('Serial port is not open or invalid');
        return;
    end
    
    % Validate setpoint is numeric
    if ~isnumeric(setpoint)
        warning('Setpoint must be a numeric value');
        return;
    end
    
    % Create message with 'S' prefix for setpoint
    setpoint_msg = ['S', num2str(setpoint)];
    
    try
        % Send data through serial port
        writeline(serialObj, setpoint_msg);
        disp(['Setpoint sent: ', setpoint_msg]);
        success = true;
    catch exception
        warning(['Failed to send setpoint: ', exception.message]);
    end
end