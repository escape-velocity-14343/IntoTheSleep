package org.firstinspires.ftc.teamcode.commands.custom;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;

public class SlowerAutonExtendCommand extends CommandBase {
    ExtensionSubsystem extend;
    double target;

    /**
     *
     * @param subsystem
     * @param target in inches
     */
    public SlowerAutonExtendCommand(ExtensionSubsystem subsystem, double target) {
        this.extend = subsystem;
        this.target = target;
        addRequirements(subsystem);
    }


    @Override
    public void initialize() {
        extend.extendInches(target);
        extend.setSuperSpeedToggle(true);
    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished() {
        return extend.isClose(target);
    }

    @Override
    public void end(boolean wasInterrupted) {
        Log.i("%9", "Extension to " + target);
        extend.setSuperSpeedToggle(false);
    }

}
