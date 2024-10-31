package org.firstinspires.ftc.teamcode.commands.custom;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;

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
    public void initialize() {
        extend.extendInches(target);
        Log.i("9", "Extension to " + target);
    }

    @Override
    public boolean isFinished() {
        return extend.isClose(target);
    }

    @Override
    public void end(boolean wasInterrupted) {
        extend.fast();
    }

}
