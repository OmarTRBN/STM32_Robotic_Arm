function success = connectSerial(app, port)
    success = false;
    try
        app.serialObj = serialport(port, 115200, "Timeout", 2);

        configureTerminator(app.serialObj, "LF");  % use newline as message delimiter
        configureCallback(app.serialObj, "terminator", @(src, ~) processSerialData(src, app));
        disp("Callback set");

        success = true;
        disp("Successfully opened port: " + port);
    catch ME
        disp("ERROR: " + ME.message);
        uialert(app.UIFigure, "Failed to open COM port: " + ME.message, "Error");
    end
end

