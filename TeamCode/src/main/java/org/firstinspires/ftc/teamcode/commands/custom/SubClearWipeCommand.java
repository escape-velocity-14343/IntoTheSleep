package org.firstinspires.ftc.teamcode.commands.custom;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.SubClearSubsystem;

public class SubClearWipeCommand extends CommandBase {

    private SubClearSubsystem subClear;
    private ElapsedTime timer;

    public SubClearWipeCommand(SubClearSubsystem subClear) {
        this.subClear = subClear;
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        subClear.wipe(true);
    }

    @Override
    public void execute() {
        if (timer.milliseconds() > IntakeConstants.subClearMillis) {
            subClear.wipe(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        subClear.close();
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > 2 * IntakeConstants.subClearMillis;
    }

}
