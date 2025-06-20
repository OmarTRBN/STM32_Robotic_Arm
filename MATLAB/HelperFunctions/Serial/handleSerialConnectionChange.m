function handleSerialConnectionChange(app, isConnect)
    if isConnect
        port = string(app.COMPortDropDown.Value);

        if port == "No COM Ports Found" || port == ""
            uialert(app.UIFigure, "No valid COM port selected!", "Error");
            app.ConnectSwitch.Value = "Off";
            return;
        end

        disconnectSerial(app);
        pause(1);

        if connectSerial(app, port)
            EnableButtons(app);
        else
            app.ConnectSwitch.Value = "Off";
        end
    else
        disconnectSerial(app);
        DisableButtons(app);
    end
end
