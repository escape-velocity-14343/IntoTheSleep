package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.Constants.SlideConstants;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class IntakeRetractCommand extends SequentialCommandGroup {
    private IntakeRetractCommand(Command... commands) {
        addCommands(commands);
    }

    public IntakeRetractCommand(WristSubsystem wrist, PivotSubsystem pivot, ExtensionSubsystem extend) {
        addCommands(
                new WristCommand(wrist, IntakeConstants.foldedPos),
                new ParallelCommandGroup(
                        new PivotCommand(pivot, PivotConstants.retractDegrees),
                        new ExtendCommand(extend, SlideConstants.minExtension).withTimeout(extend.getReasonableExtensionMillis(0))
                )

        );
    }

    public static IntakeRetractCommand newWithWristPos(WristSubsystem wrist, PivotSubsystem pivot, ExtensionSubsystem extend, double wristPos) {
        return new IntakeRetractCommand(
                new ParallelCommandGroup(
                        new PivotCommand(pivot, PivotConstants.retractDegrees),
                        new ExtendCommand(extend, SlideConstants.minExtension).withTimeout(extend.getReasonableExtensionMillis(0))
                ),
                new WristCommand(wrist, wristPos).whenFinished(() -> Log.i("5", "Retract command"))
        );
    }
}
