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

/**
 * Flips the wrist down after a SubPosReadyCommand.
 */
public class SubPosCommand extends SequentialCommandGroup {
    ExtensionSubsystem extension;

    private SubPosCommand(ExtensionSubsystem extension, Command... commands) {
        super(commands);
        this.extension = extension;

    }

    public SubPosCommand(ExtensionSubsystem extension, WristSubsystem wrist, IntakeSubsystem intake) {
        addCommands(
                //new ExtendCommand(extension, 0)
                //        .withTimeout(extension.getReasonableExtensionMillis(0)),
                new WristCommand(wrist, IntakeConstants.groundPos)
                        .alongWith(new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 1))
        );
        // must require extension because manual control must use it, so this ensures any other commands using extension get interrupted
        addRequirements(extension);
        this.extension = extension;
    }

    @Override
    public void end(boolean interrupted) {
        extension.setManualControl(true);
        Log.i("6", "Sub Pos Command, Interrupted: " + interrupted);
    }

    @Deprecated
    public static Command newWithExtension(ExtensionSubsystem extension, WristSubsystem wrist, IntakeSubsystem intake, double inches) {
        return new SubPosCommand(extension,
                new ExtendCommand(extension, inches)
                        .withTimeout(extension.getReasonableExtensionMillis(inches)),
                new WristCommand(wrist, IntakeConstants.groundPos)
                        .alongWith(new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 1))
        );
    }
}
