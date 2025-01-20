package org.firstinspires.ftc.teamcode.commands.custom;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.Constants.SlideConstants;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class SpecimenRaiseCommand extends SequentialCommandGroup {
    public SpecimenRaiseCommand(PivotSubsystem pivot, ExtensionSubsystem extension, WristSubsystem wrist) {
        addCommands(
                new ParallelCommandGroup(
                    new WristCommand(wrist, IntakeConstants.specimenReadyPos),
                    new PivotCommand(pivot, PivotConstants.specimenIntakeAngle),
                    new ExtendCommand(extension, SlideConstants.specimenRaisePosition)
                ),
                // ensure slides are retracted
                new SlowerAutonExtendCommand(extension, SlideConstants.specimenRaisePosition - 2).withTimeout(250),
                // prevent stalling
                new ExtendCommand(extension, SlideConstants.specimenRaisePosition).withTimeout(10)
        );
    }
}
