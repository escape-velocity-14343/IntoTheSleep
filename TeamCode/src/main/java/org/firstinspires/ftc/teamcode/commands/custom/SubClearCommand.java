package org.firstinspires.ftc.teamcode.commands.custom;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.SubClearSubsystem;

public class SubClearCommand extends CommandBase {

    private SubClearSubsystem subClear;
    private ElapsedTime timer;

    {
        timer = new ElapsedTime();
    }

    public SubClearCommand(SubClearSubsystem subClear) {
        this.subClear = subClear;
        addRequirements(subClear);
    }

    @Override
    public void initialize() {
        subClear.open();
        timer.reset();
    }

    @Override
    public void execute() {
        if (timer.milliseconds() > IntakeConstants.subClearMillis) {
            subClear.close();
        }
    }

    @Override
    public void end(boolean interrupted) {
        subClear.close();
    }

    public boolean isFinished() {
        return timer.milliseconds() > IntakeConstants.subClearMillis * 2;
    }

}
