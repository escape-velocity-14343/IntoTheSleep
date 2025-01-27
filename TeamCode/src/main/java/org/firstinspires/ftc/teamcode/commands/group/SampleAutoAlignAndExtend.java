package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.SlideConstants;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.vision.ColorSensorProcessor;

public class SampleAutoAlignAndExtend extends CommandBase {
    CameraSubsystem cam;
    DefaultGoToPointCommand gtpc;
    private PinpointSubsystem pinpointSubsystem;
    private ExtensionSubsystem extensionSubsystem;
    private IntakeSubsystem intake;
    private ElapsedTime time = new ElapsedTime();
    private ElapsedTime extensionLimitTime = new ElapsedTime();
    private ElapsedTime wrongColorTime = new ElapsedTime();
    private boolean isWrongColor = false;
    private boolean seen = false;
    private boolean reachedMaxExtension = false;
    public SampleAutoAlignAndExtend(CameraSubsystem camera, DefaultGoToPointCommand gtpc, PinpointSubsystem pinpoint, ExtensionSubsystem extensionSubsystem, IntakeSubsystem intake) {
        addRequirements(camera);
        cam = camera;
        this.gtpc = gtpc;
        this.pinpointSubsystem = pinpoint;
        this.extensionSubsystem = extensionSubsystem;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        cam.setEnabled(true);
        time.reset();
        extensionSubsystem.setManualControl(true);
    }
    @Override
    public void execute() {
        Log.i("autoalign", "yellow is: " + cam.isYellow());
        Log.i("autoalign", "timer: " + time.milliseconds());
        gtpc.setTarget(new Pose2d(gtpc.getTargetX(), gtpc.getTargetY(), Rotation2d.fromDegrees(pinpointSubsystem.getPose().getRotation().getDegrees() + cam.getPixelPos() * IntakeConstants.autoAlignP)));

        if (cam.getColor() == (AutoConstants.alliance == AutoConstants.Alliance.RED ? ColorSensorProcessor.ColorType.BLUE : ColorSensorProcessor.ColorType.RED)) {
            intake.setIntakeSpeed(-1);
            isWrongColor = true;
            wrongColorTime.reset();
        } else if (isWrongColor) {
            if (wrongColorTime.seconds() > 0.5 || extensionSubsystem.getCurrentInches() < 6.0) {
                isWrongColor = false;
                intake.setClawer(IntakeConstants.openPos * IntakeConstants.autoIntakeClawLerp + IntakeConstants.singleIntakePos * (1 - IntakeConstants.autoIntakeClawLerp));
            }
            extensionSubsystem.setPower(-0.4);
            intake.setClawer(IntakeConstants.openPos);
            intake.setIntakeSpeed(-1);
        } else {
            intake.setIntakeSpeed(1);
            extensionSubsystem.setPower(Range.clip(Math.cos((cam.getPixelPos())*0.01)*SlideConstants.visionP, 0,1) * extensionSubsystem.getVoltageMult());
        }
    }
    @Override
    public boolean isFinished() {

        if (extensionSubsystem.getCurrentInches() > SlideConstants.submersibleIntakeMaxExtension - 0.1) {
            if (!reachedMaxExtension) {
                reachedMaxExtension = true;
                extensionLimitTime.reset();
            } else if (extensionLimitTime.seconds() > 0.4) {
                intake.setClawer(IntakeConstants.closedPos);
            } else if (extensionLimitTime.seconds() > 0.5) {
                return true;
            }
        } else {
            reachedMaxExtension = false;
        }

        if (cam.isYellow() || cam.getColor() == (AutoConstants.alliance == AutoConstants.Alliance.BLUE ? ColorSensorProcessor.ColorType.BLUE : ColorSensorProcessor.ColorType.RED)) {
            if (!seen) {
                seen = true;
                time.reset();
            }
            else if (time.milliseconds()>25) {
                return true;
            }
        } else {
            seen = false;
            return false;
        }
        return false;
    }
    @Override
    public void end (boolean wasInterrupted) {
        Log.i("autoalign", "autoalign done, color: " + (cam.isYellow() ? "yellow" : "blue"));
        //cam.setEnabled(false);
        extensionSubsystem.setManualControl(false);
    }

}
