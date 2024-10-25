package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;

public class SingleLEDCommand extends LEDCommand {
    private final String ledName;

    public SingleLEDCommand(LEDSubsystem leds, boolean enable, String ledName) {
        super(leds, enable);
        this.ledName = ledName;
    }

    @Override
    public void initialize() {
        ledSubsystem.setLightState(enable, ledName);
    }
}
