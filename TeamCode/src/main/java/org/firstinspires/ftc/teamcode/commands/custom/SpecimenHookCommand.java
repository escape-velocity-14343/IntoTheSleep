package org.firstinspires.ftc.teamcode.commands.custom;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.Constants.SlideConstants;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class SpecimenHookCommand extends SequentialCommandGroup {
    public SpecimenHookCommand(PivotSubsystem pivot, ExtensionSubsystem extend, WristSubsystem wrist, IntakeSubsystem intake) {
        addCommands(
                new IntakeControlCommand(intake, 1, 0.5),
                new WristCommand(wrist, IntakeConstants.scoringPos).alongWith(
                        new ExtendCommand(extend, SlideConstants.specimenHighRaisePosition).withTimeout(400)
                ),
                new WaitCommand(250),
                new ParallelCommandGroup(
                    new PivotCommand(pivot, PivotConstants.specimenTopBarAngle),
                    new ExtendCommand(extend, SlideConstants.specimenHookPosition)
                ).withTimeout(300),
                new IntakeClawCommand(intake, IntakeConstants.singleIntakePos),
                new ParallelCommandGroup(
                        new PivotCommand(pivot, PivotConstants.specimenTopBarAngle),
                        new ExtendCommand(extend, SlideConstants.specimenHookPosition)
                ).withTimeout(300),
                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 0)
        );
    }
}
