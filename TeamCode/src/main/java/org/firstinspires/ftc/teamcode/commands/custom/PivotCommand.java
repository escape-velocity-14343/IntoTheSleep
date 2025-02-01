package org.firstinspires.ftc.teamcode.commands.custom;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;

public class PivotCommand extends CommandBase {
    PivotSubsystem pivotSubsystem;
    double target;

    /**
     *
     * @param pivotSubsystem
     * @param target in degrees
     */

    public PivotCommand(PivotSubsystem pivotSubsystem, double target) {
        this.pivotSubsystem = pivotSubsystem;
        this.target = target;
        addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize(){
        pivotSubsystem.setTarget(target);
    }

    @Override
    public void execute() {
        //Log.v("%12", "Pivot Debug Target: " + target);
        //Log.v("%12", "Pivot Debug Position: " + pivotSubsystem.getCurrentPosition());
        //Log.v("%12", "Pivot Debug isClose: " + isFinished());
    }

    @Override
    public boolean isFinished(){
        return pivotSubsystem.isClose(target);
    }

    @Override
    public void end(boolean wasInterrupted) {
        Log.i("%7", "Pivot reached target: " + target);
        pivotSubsystem.stop();
    }
}
