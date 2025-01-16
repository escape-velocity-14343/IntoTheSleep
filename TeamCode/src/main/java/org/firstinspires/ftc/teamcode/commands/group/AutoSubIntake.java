package org.firstinspires.ftc.teamcode.commands.group;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.vision.ColorSensorProcessor;

public class AutoSubIntake extends SequentialCommandGroup {

    public AutoSubIntake(ExtensionSubsystem extension, WristSubsystem wrist, CameraSubsystem cam, DefaultGoToPointCommand gtpc, IntakeSubsystem intake, PinpointSubsystem pinpoint, PivotSubsystem pivot) {
        addCommands(
                new ExtendCommand(extension, 5),
                new WristCommand(wrist, IntakeConstants.groundPos).alongWith(
                        new IntakeControlCommand(intake, IntakeConstants.singleIntakePos-0.025, 1)),
                new WaitCommand(100),
                //new SampleAutoAlign(cam, gtpc, pinpoint).deadlineWith(
                //        new AutonExtendCommand(extension, SlideConstants.submersibleIntakeMaxExtension)).withTimeout(2000),
                new SampleAutoAlignAndExtend(cam, gtpc, pinpoint, extension, intake).withTimeout(2000),

                new ConditionalCommand(
                        new IntakeControlCommand(intake, IntakeConstants.closedPos, 0.5),
                        new IntakeControlCommand(intake, IntakeConstants.openPos, -0.5),
                        () -> cam.isYellow() || cam.getColor() == (AutoConstants.alliance == AutoConstants.Alliance.BLUE ? ColorSensorProcessor.ColorType.BLUE : ColorSensorProcessor.ColorType.RED)),


                new IntakeControlCommand(intake, IntakeConstants.closedPos, 0).alongWith(
                        new IntakeRetractCommand(wrist, pivot, extension))
        );
    }

}
