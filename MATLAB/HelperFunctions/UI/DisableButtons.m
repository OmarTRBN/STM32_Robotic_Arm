function DisableButtons(app)
    app.ToggleLEDButton.Enable = "off";
    app.M1_Button.Enable = "off";
    app.M2_Button.Enable = "off";

    app.COMPortDropDown.Enable = "on";
end