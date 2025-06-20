function processSerialData(src, app)
    disp("RX: " + readline(src));
    try
        line = readline(src);  % get incoming line
        if startsWith(line, "<d>")
            debugMsg = extractAfter(line, "<d>");
            appendToTextArea(app, debugMsg);
        elseif startsWith(line, "<s>")
            % Extract sensor value using regex
            token = regexp(line, '<s>(\d+)<e>', 'tokens');
            if ~isempty(token)
                val = str2double(token{1}{1});
                plotSensorValue(app, val);
            end
        end
    catch ME
        disp("Serial read error: " + ME.message);
    end
end
