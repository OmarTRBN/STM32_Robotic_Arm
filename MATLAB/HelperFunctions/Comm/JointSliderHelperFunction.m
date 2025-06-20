function JointSliderHelperFunction(app, event)
    source = event.Source;

    if source == app.Joint1Slider
        motorID = '0';
    elseif source == app.Joint2Slider
        motorID = '1';
    else
        return;  % unknown source
    end

    state = source.Value;
    stateStr = num2str(round(state));  % Convert numeric slider value to string
    msg = ['M', 'R', motorID, ',', stateStr, newline];  % Add delimiter and newline
    write(app.serialObj, msg, 'char');

end