package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.Constants.SlideConstants;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

import java.util.function.Function;

public class IntakeRetractReversedCommand extends SequentialCommandGroup {
    private IntakeRetractReversedCommand(Command... commands) {
        addCommands(commands);
    }

    public IntakeRetractReversedCommand(WristSubsystem wrist, PivotSubsystem pivot, ExtensionSubsystem extend) {
        addCommands(

                new ParallelCommandGroup(
                    new WaitUntilCommand(() -> pivot.getCurrentPosition() > 5)
                            .andThen(new WristCommand(wrist, IntakeConstants.foldedPos), new ExtendCommand(extend, SlideConstants.minExtension).withTimeout(extend.getReasonableExtensionMillis(SlideConstants.minExtension))),
                    new PivotCommand(pivot, PivotConstants.reversedRetractDegrees)
                )

        );
    }

    public static IntakeRetractReversedCommand newWithWristPos(WristSubsystem wrist, PivotSubsystem pivot, ExtensionSubsystem extend, double wristPos) {
        return new IntakeRetractReversedCommand(
                new ParallelCommandGroup(
                        new WaitUntilCommand(() -> pivot.getCurrentPosition() > 5)
                                .andThen(new WristCommand(wrist, wristPos), new ExtendCommand(extend, SlideConstants.minExtension).withTimeout(extend.getReasonableExtensionMillis(SlideConstants.minExtension))),
                        new PivotCommand(pivot, PivotConstants.reversedRetractDegrees)
                )
        );
    }
}
