function disconnectSerial(app)
    if ~isempty(app.serialObj)
        delete(app.serialObj);
        app.serialObj = [];
    end
end
