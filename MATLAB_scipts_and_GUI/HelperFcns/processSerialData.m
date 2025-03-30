function processSerialData(src, app)
    data = readline(src);
    app.TextArea.Value = data;  % Display the data in TextArea
    
    % If the data can be converted to a number, show it in EditField
    try
        numVal = str2double(data);
        if ~isnan(numVal)
            app.EditField.Value = numVal;
        end
    catch
        % If conversion fails, just keep the text display
    end
end