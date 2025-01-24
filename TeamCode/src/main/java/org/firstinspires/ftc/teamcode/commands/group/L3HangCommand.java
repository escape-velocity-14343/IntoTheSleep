package org.firstinspires.ftc.teamcode.commands.group;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

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
    public L3HangCommand (AscentSubsytem hang, PivotSubsystem pivot, WristSubsystem wrist, ExtensionSubsystem extensionSubsystem, MecanumDriveSubsystem drive) {
        addRequirements(hang, pivot);
        addCommands(
                new WristCommand(wrist, IntakeConstants.foldedPos),
                new InstantCommand(() -> pivot.setManualControl(true)),
                new RunCommand(()->pivot.setPower(-1),pivot).interruptOn(()->pivot.getCurrentPosition()<1),
                new InstantCommand(() -> pivot.setManualControl(false)),
                new PivotCommand(pivot, 0),

                new HangStateCommand(hang, AscentSubsytem.PTOMode.FREEFLOAT),
                new WaitCommand(500),
                new HangStateCommand(hang, AscentSubsytem.PTOMode.ENGAGED),
                new L3HangRetract(hang, drive),
                new ExtendCommand(extensionSubsystem, SlideConstants.submersibleIntakeMaxExtension),
                new InstantCommand(()->hang.move(0.1))
        );
    }
}
