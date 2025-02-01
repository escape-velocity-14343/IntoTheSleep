package org.firstinspires.ftc.teamcode.commands.group;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.SlideConstants;
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

public class L3HangCommand extends SequentialCommandGroup {
    public L3HangCommand(AscentSubsytem hang, PivotSubsystem pivot, WristSubsystem wrist, ExtensionSubsystem extensionSubsystem, MecanumDriveSubsystem drive) {
        addRequirements(hang, pivot);
        addCommands(
                new WristCommand(wrist, IntakeConstants.foldedPos),
                new InstantCommand(() -> pivot.setManualControl(true)),
                new InstantCommand(() -> pivot.setPower(-1), pivot),
                new WaitUntilCommand(() -> pivot.getCurrentPosition() < 1),
                new HangStateCommand(hang, AscentSubsytem.PTOMode.FREEFLOAT),
                //new InstantCommand(()->pivot.setPower(-0.7),pivot),
                new WaitCommand(1500).deadlineWith(
                        new ExtendCommand(extensionSubsystem, 1)
                ),
                new HangStateCommand(hang, AscentSubsytem.PTOMode.ENGAGED),
                new InstantCommand(() -> pivot.setPower(0), pivot),
                new InstantCommand(() -> pivot.setManualControl(false)),
                new PivotCommand(pivot, 0).interruptOn(() -> true),
                new L3HangRetract(hang, drive)

        );
    }
}
