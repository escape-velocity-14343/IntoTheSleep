package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    public boolean isFinished(){
        return pivotSubsystem.isClose(target);
    }

}
