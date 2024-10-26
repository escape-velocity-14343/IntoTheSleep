package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;

public class LEDCommand extends CommandBase {
    protected final LEDSubsystem ledSubsystem;
    protected final boolean enable;

    public LEDCommand(LEDSubsystem leds, boolean enable) {
        ledSubsystem = leds;
        this.enable = enable;
    }

    @Override
    public void initialize() {
        ledSubsystem.setLightState(enable);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
