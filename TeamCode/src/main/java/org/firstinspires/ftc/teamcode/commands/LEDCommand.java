package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;

import kotlin.jvm.functions.Function3;

public class LEDCommand extends CommandBase {
    LEDSubsystem ledSubsystem;
    boolean state = false;

    private LEDCommand(LEDSubsystem leds, boolean enable) {
        ledSubsystem = leds;
        state = enable;
    }

    @Override
    public void initialize(){
        ledSubsystem.lightSwitch(state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
