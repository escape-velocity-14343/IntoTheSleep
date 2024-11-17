package org.firstinspires.ftc.teamcode.commands.group;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class SubClearCommand extends SequentialCommandGroup {
    public SubClearCommand(ExtensionSubsystem extend, PivotSubsystem pivot, WristSubsystem wrist) {
        addCommands(
                new ExtendCommand(extend, 2).withTimeout(250).alongWith(
                        new WristCommand(wrist, IntakeConstants.halfFoldPos)),
                new WaitCommand(50),
                new ExtendCommand(extend, 8),
                new WristCommand(wrist, IntakeConstants.groundPos),
                new ExtendCommand(extend, 2).withTimeout(300));

    }
}
