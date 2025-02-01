package org.firstinspires.ftc.teamcode.commands.group;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.HangStateCommand;
import org.firstinspires.ftc.teamcode.commands.custom.L3HangRetract;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.subsystems.AscentSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class L3ReleaseCommand extends SequentialCommandGroup {
    public L3ReleaseCommand(AscentSubsytem hang, PivotSubsystem pivot, WristSubsystem wrist, ExtensionSubsystem extensionSubsystem) {
        addRequirements(hang, pivot,extensionSubsystem);
        addCommands(
                new InstantCommand(()->pivot.setManualControl(true)),
                new RunCommand(()->pivot.setPower(-1)).interruptOn(()->pivot.getCurrentPosition()<2),

                new HangStateCommand(hang, AscentSubsytem.PTOMode.FREEFLOAT),
                new ExtendCommand(extensionSubsystem, 4)

                //new InstantCommand(()->pivot.setPower(-0.7),pivot),
        );
    }
}
