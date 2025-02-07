package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
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

public class RetractCommand extends SequentialCommandGroup {
    private RetractCommand(Command... commands) {
        addCommands(commands);
    }

    public RetractCommand(WristSubsystem wrist, PivotSubsystem pivot, ExtensionSubsystem extend) {

        addCommands(
                new WristCommand(wrist, IntakeConstants.groundPos),
                new ParallelCommandGroup(
                        new WaitUntilCommand(() -> extend.getCurrentInches() < 10).andThen(new PivotCommand(pivot, PivotConstants.retractDegrees).alongWith(
                                        new WristCommand(wrist, IntakeConstants.foldedPos)
                                )
                        ),
                        new ExtendCommand(extend, SlideConstants.minExtension)
                ),
                new WristCommand(wrist, IntakeConstants.foldedPos).whenFinished(() -> Log.i("5", "Retract command"))
        );

    }

    public static RetractCommand newWithWristPos(WristSubsystem wrist, PivotSubsystem pivot, ExtensionSubsystem extend, double wristPos) {
        return new RetractCommand(
                new WristCommand(wrist, wristPos),
                new ParallelCommandGroup(
                        new WaitUntilCommand(() -> extend.getCurrentInches() < 10).andThen(new PivotCommand(pivot, PivotConstants.retractDegrees).alongWith(
                                        new WristCommand(wrist, IntakeConstants.foldedPos)
                                )
                        ),
                        new ExtendCommand(extend, SlideConstants.minExtension)
                ),
                new WristCommand(wrist, IntakeConstants.foldedPos).whenFinished(() -> Log.i("%5", "Retract command"))
        );
    }
}
