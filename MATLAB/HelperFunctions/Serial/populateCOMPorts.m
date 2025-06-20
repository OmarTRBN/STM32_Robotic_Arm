function populateCOMPorts(app)
    comPorts = serialportlist("available");
    if isempty(comPorts)
        app.COMPortDropDown.Items = {"No COM Ports Found"};
    else
        app.COMPortDropDown.Items = comPorts;
    end
end
