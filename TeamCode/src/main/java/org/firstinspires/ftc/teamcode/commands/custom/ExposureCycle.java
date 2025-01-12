package org.firstinspires.ftc.teamcode.commands.custom;


import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;

public class ExposureCycle extends CommandBase {
    CameraSubsystem cam;
    int exposureMillis = 5;
    boolean lastExposureSuccess = false;
    public ExposureCycle(CameraSubsystem camera) {
        cam = camera;
        addRequirements(cam);
    }
    @Override
    public void initialize() {
        cam.setExposure(exposureMillis);
    }
    @Override
    public void execute() {
        if (lastExposureSuccess) {
            cam.saveFrame("Exposure: " + exposureMillis + "ms");
            exposureMillis+=5;
        }
        lastExposureSuccess = cam.setExposure(exposureMillis);
    }
    @Override
    public void end(boolean wasInterrupted) {
        //reset to original exposure (ftcdash)
        cam.setExposure();
    }
    @Override
    public boolean isFinished() {
        return exposureMillis >= 100;
    }
}
