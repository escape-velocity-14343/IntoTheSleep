package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class IntakePosCommand extends SequentialCommandGroup {
    public IntakePosCommand(ExtensionSubsystem extend, PivotSubsystem pivot, WristSubsystem wrist) {
        addCommands(
                new ParallelCommandGroup(
                    new WristCommand(wrist, IntakeConstants.foldedPos),
                    new ExtendCommand(extend,0)),
                new PivotCommand(pivot, 0),
                new WristCommand(wrist, 0.5));
    }
}
