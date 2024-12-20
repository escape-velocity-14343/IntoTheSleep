package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;

import com.arcrobotics.ftclib.command.LogCatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

// AUTO ONLY
public class IntakePosCommand extends SequentialCommandGroup {
    public IntakePosCommand(ExtensionSubsystem extend, PivotSubsystem pivot, WristSubsystem wrist) {
        addCommands(
                //new ParallelCommandGroup(
                new ExtendCommand(extend,0)
                        .alongWith(
                                new WaitUntilCommand(() -> extend.getCurrentInches() < 25).andThen(
                                        new WristCommand(wrist, IntakeConstants.halfFoldPos),
                                        new PivotCommand(pivot, PivotConstants.retractDegrees))),
                new WristCommand(wrist, IntakeConstants.groundPos).whenFinished(() -> Log.i("intake pos command", "pivot pos: " + pivot.getCurrentPosition())));
    }
}
