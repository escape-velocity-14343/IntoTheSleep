package org.firstinspires.ftc.teamcode.commands.group;

import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.SlideConstants;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class BucketPosCommand extends SequentialCommandGroup {
    public BucketPosCommand(ExtensionSubsystem extension, PivotSubsystem pivot, WristSubsystem wrist) {
        addCommands(
                new WristCommand(wrist, IntakeConstants.scoringPos),
                new ExtendCommand(extension, 1),
                new ParallelDeadlineGroup(
                        new PivotCommand(pivot, 90),
                        new ParallelDeadlineGroup(new WaitCommand(250), new ExtendCommand(extension, SlideConstants.maxExtension).withTimeout(1000)))
        );
    }
}
