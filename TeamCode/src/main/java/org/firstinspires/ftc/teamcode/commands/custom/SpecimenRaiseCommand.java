package org.firstinspires.ftc.teamcode.commands.custom;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.Constants.SlideConstants;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class SpecimenRaiseCommand extends ParallelCommandGroup {
    public SpecimenRaiseCommand(PivotSubsystem pivot, ExtensionSubsystem extension, WristSubsystem wrist) {
        addCommands(
                new WristCommand(wrist, IntakeConstants.specimenReadyPos),
                new PivotCommand(pivot, PivotConstants.specimenIntakeAngle),
                new ExtendCommand(extension, SlideConstants.specimenRaisePosition)
        );
    }
}
