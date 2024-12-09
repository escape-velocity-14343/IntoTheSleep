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
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.vision.ColorSensorProcessor;

public class SampleAutoAlignAndExtend extends CommandBase {
    CameraSubsystem cam;
    DefaultGoToPointCommand gtpc;
    private PinpointSubsystem pinpointSubsystem;
    private ExtensionSubsystem extensionSubsystem;
    private ElapsedTime time = new ElapsedTime();
    private boolean seen = false;
    public SampleAutoAlignAndExtend(CameraSubsystem camera, DefaultGoToPointCommand gtpc, PinpointSubsystem pinpoint, ExtensionSubsystem extensionSubsystem) {
        addRequirements(camera);
        cam = camera;
        this.gtpc = gtpc;
        this.pinpointSubsystem = pinpoint;
        this.extensionSubsystem = extensionSubsystem;
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
        extensionSubsystem.setPower(Range.clip(Math.cos((cam.getPixelPos())*0.01)*SlideConstants.visionP, 0,0.5));
    }
    @Override
    public boolean isFinished() {

        if (cam.isYellow() || cam.getColor() == (AutoConstants.alliance == AutoConstants.Alliance.BLUE ? ColorSensorProcessor.ColorType.BLUE : ColorSensorProcessor.ColorType.RED)) {
            if (!seen) {
                seen = true;
                time.reset();
            }
            else if (time.milliseconds()>100) {
                return true;
            }
        }
        else {
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
