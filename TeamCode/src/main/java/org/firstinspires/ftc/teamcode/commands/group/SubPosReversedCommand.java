package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class SubPosReversedCommand extends SequentialCommandGroup {
    ExtensionSubsystem extension;

    private SubPosReversedCommand(ExtensionSubsystem extension, Command... commands) {
        super(commands);
        this.extension = extension;
    }



    public SubPosReversedCommand(ExtensionSubsystem extension, WristSubsystem wrist, IntakeSubsystem intake, PivotSubsystem pivot) {
        addCommands(
                new ExtendCommand(extension, 10),
                new WristCommand(wrist, IntakeConstants.groundPos),
                new IntakeControlCommand(intake, IntakeConstants.backSinglePos, -1),
                new PivotCommand(pivot, PivotConstants.retractDegrees)
        );
        this.extension = extension;
    }

    @Override
    public void end(boolean interrupted) {
        extension.setManualControl(true);
        Log.i("6", "Sub Pos Command, Interrupted: " + interrupted);
    }

    public static Command newWithExtension(ExtensionSubsystem extension, WristSubsystem wrist, IntakeSubsystem intake, PivotSubsystem pivot, double inches) {
        return new SubPosReversedCommand(extension,
                new ExtendCommand(extension, 10),
                new WristCommand(wrist, IntakeConstants.groundPos),
                new IntakeControlCommand(intake, IntakeConstants.backSinglePos, -1),
                new PivotCommand(pivot, PivotConstants.retractDegrees)
        );
    }
}
