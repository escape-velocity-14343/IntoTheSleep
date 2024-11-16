package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

public class SampleAutoAlign extends CommandBase {
    CameraSubsystem cam;
    DefaultGoToPointCommand gtpc;
    private PinpointSubsystem pinpointSubsystem;
    private ElapsedTime time = new ElapsedTime();
    private boolean seen = false;
    public SampleAutoAlign(CameraSubsystem camera, DefaultGoToPointCommand gtpc, PinpointSubsystem pinpoint) {
        addRequirements(camera);
        cam = camera;
        this.gtpc = gtpc;
        this.pinpointSubsystem = pinpoint;
    }
    @Override
    public void initialize() {
        cam.setEnabled(true);
        time.reset();
    }
    @Override
    public void execute() {
        Log.i("autoalign", "yellow is: " + cam.getYellow());
        Log.i("autoalign", "timer: " + time.milliseconds());
        gtpc.setTarget(new Pose2d(gtpc.getTargetX(), gtpc.getTargetY(), Rotation2d.fromDegrees(pinpointSubsystem.getPose().getRotation().getDegrees() + cam.getPixelPos() * IntakeConstants.autoAlignP)));
    }
    @Override
    public boolean isFinished() {

        if (cam.getYellow()) {
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
        cam.setEnabled(false);
    }

}
