function appendToTextArea(app, msg)
    if ischar(msg)
        msg = string(msg);  % normalize input
    end
    app.TextArea.Value = msg;  % OVERWRITE previous content
    drawnow limitrate;         % update UI responsively
end
