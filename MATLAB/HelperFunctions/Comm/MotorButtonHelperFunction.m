function MotorButtonHelperFunction(app, event)
    source = event.Source;  % Which button triggered it?

    if source == app.M1_Button
        motorID = '0';
    elseif source == app.M2_Button
        motorID = '1';
    else
        return;  % unknown source
    end

    state = source.Value;
    msg = ['M', 'S', motorID, char(48 + state), newline];
    write(app.serialObj, msg, 'char');
end