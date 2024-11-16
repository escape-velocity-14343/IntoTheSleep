package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;

import com.arcrobotics.ftclib.command.InstantCommand;
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

public class BucketPosReversedCommand extends SequentialCommandGroup {
    public BucketPosReversedCommand(ExtensionSubsystem extension, PivotSubsystem pivot, WristSubsystem wrist) {
        addCommands(
                new WristCommand(wrist, IntakeConstants.scoringPosReversed),
                //new ExtendCommand(extension, 1),
                new ParallelCommandGroup(
                        // minus two to prevent it from overshooting
                        new PivotCommand(pivot, PivotConstants.frontOuttakePosition),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> pivot.getCurrentPosition() > PivotConstants.outtakeExtendDegrees),
                                new ExtendCommand(extension, SlideConstants.maxExtension).withTimeout(1000)
                        )
                ),
                new InstantCommand(() -> Log.i("2", "BucketPos End"))
        );
    }
}
