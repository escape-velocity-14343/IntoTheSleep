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
    private SubPosReversedCommand(Command... commands) {
        super(commands);
    }

    public SubPosReversedCommand(ExtensionSubsystem extension, WristSubsystem wrist, IntakeSubsystem intake, PivotSubsystem pivot) {
        addCommands(
                new ExtendCommand(extension, 10),
                new WristCommand(wrist, IntakeConstants.groundPos),
                new IntakeControlCommand(intake, IntakeConstants.backSinglePos, -1),
                new InstantCommand(() -> extension.setManualControl(true)).alongWith(
                        new PivotCommand(pivot, PivotConstants.retractDegrees))
                .andThen(new InstantCommand(() -> extension.setManualControl(true)))
                .whenFinished(() -> Log.i("6", "Sub Pos Command"))
        );
    }

    public static SubPosReversedCommand newWithExtension(ExtensionSubsystem extension, WristSubsystem wrist, IntakeSubsystem intake, PivotSubsystem pivot, double inches) {
        return new SubPosReversedCommand(
                new ExtendCommand(extension, inches),
                new WristCommand(wrist, IntakeConstants.groundPos),
                new IntakeControlCommand(intake, IntakeConstants.backSinglePos, -1),
                new InstantCommand(() -> extension.setManualControl(true))
                        .alongWith(
                            new PivotCommand(pivot, PivotConstants.retractDegrees))
                            .andThen(new InstantCommand(() -> extension.setManualControl(true)))
                            .whenFinished(() -> Log.i("6", "Sub Pos Command")
                        )
        );
    }
}
