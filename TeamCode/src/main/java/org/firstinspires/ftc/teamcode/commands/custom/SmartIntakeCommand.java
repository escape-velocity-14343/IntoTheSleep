package org.firstinspires.ftc.teamcode.commands.custom;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.vision.ColorSensorProcessor;

public class SmartIntakeCommand extends InstantCommand {

    public SmartIntakeCommand(IntakeSubsystem intake, CameraSubsystem cam) {
        super(() ->
        {
           // if we detect a sample
            if (intake.getDSensorSupplier()) {
                // reject if bad
                if (cam.getColor() == (AutoConstants.alliance == AutoConstants.Alliance.BLUE ? ColorSensorProcessor.ColorType.RED : ColorSensorProcessor.ColorType.BLUE)) {
                    intake.setIntakeSpeed(-1);
                // otherwise autoclose
                } else {
                    intake.setIntakeSpeed(1);
                    intake.setClawer(IntakeConstants.closedPos);
                }
            } else {
                intake.setIntakeSpeed(1);
            }


        });

        addRequirements(intake);
    }

}
