package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.SlideConstants;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeSpinCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class SubPosCommand extends SequentialCommandGroup {
    private SubPosCommand(Command... commands) {
        super(commands);
    }

    public SubPosCommand(ExtensionSubsystem extension, WristSubsystem wrist, IntakeSubsystem intake) {
        addCommands(
                new ExtendCommand(extension, 0)
                        .withTimeout(extension.getReasonableExtensionMillis(0)),
                new WristCommand(wrist, IntakeConstants.groundPos)
                        .alongWith(new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 1))
                        .andThen(new InstantCommand(() -> extension.setManualControl(true)))
                        .whenFinished(() -> Log.i("6", "Sub Pos Command"))
        );
    }

    public static SubPosCommand newWithExtension(ExtensionSubsystem extension, WristSubsystem wrist, IntakeSubsystem intake, double inches) {
        return new SubPosCommand(
                new ExtendCommand(extension, inches)
                        .withTimeout(extension.getReasonableExtensionMillis(inches)),
                new WristCommand(wrist, IntakeConstants.groundPos)
                        .alongWith(new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 1))
                        .andThen(new InstantCommand(() -> extension.setManualControl(true)))
                        .whenFinished(() -> Log.i("6", "Sub Pos Command"))
        );
    }
}
