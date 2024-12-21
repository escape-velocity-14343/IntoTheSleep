package org.firstinspires.ftc.teamcode.commands.group;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.Constants.SlideConstants;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

/**
 * Command for extending slides out and half flipping claw down, but not yet fully bringing the claw down.
 */
public class SubPosReadyCommand extends SequentialCommandGroup {

    public SubPosReadyCommand(ExtensionSubsystem extension, PivotSubsystem pivot, WristSubsystem wrist, IntakeSubsystem intake, double extendInches) {

        // validate that this extension is within extension limit
        assert extendInches < SlideConstants.submersibleIntakeMaxExtension;

        addCommands(
                new ParallelCommandGroup(
                        // extend to the target
                        new ExtendCommand(extension, extendInches).withTimeout(extension.getReasonableExtensionMillis(extendInches)),

                        // wait until extension is below a certain threshold to stay within extension limit, then pivot to neutral
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> extension.getCurrentInches() < SlideConstants.maxPivotExtension),
                                new PivotCommand(pivot, PivotConstants.neutralPos)
                        )

                ),
                // flip down wrist to a ready position
                new WristCommand(wrist, IntakeConstants.intakeReadyPos)
        );
    }


}
