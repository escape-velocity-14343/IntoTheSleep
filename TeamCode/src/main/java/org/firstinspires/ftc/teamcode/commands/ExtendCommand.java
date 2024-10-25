package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;

public class ExtendCommand extends CommandBase {
    ExtensionSubsystem extend;
    double target;

    /**
     *
     * @param subsystem
     * @param target in inches
     */
    public ExtendCommand(ExtensionSubsystem subsystem, double target) {
        this.extend = subsystem;
        this.target = target;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        extend.extendInches(target);
        Log.println(Log.VERBOSE, "the lienar slides", "target: " + target);
    }

    @Override
    public boolean isFinished(){
        return extend.isClose(target);
    }
    @Override
    public void end(boolean wasInterrupted) {
        extend.stop();
    }

}
