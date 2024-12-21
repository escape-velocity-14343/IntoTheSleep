package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.lib.Localizer;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.lib.VelocityCompensatedSquIDController;
import org.firstinspires.ftc.teamcode.opmode.test.VelocityCompensatedSquIDTest;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OTOSSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

import java.util.function.DoubleSupplier;

@Config
public class DefaultGoToPointCommand extends CommandBase {
    public PIDController xPID = new PIDController(0,0,0);
    public PIDController yPID = new PIDController(0, 0,0);
    public PIDController headingPID = new PIDController(0,0,0);

    public VelocityCompensatedSquIDController positionSquID = new VelocityCompensatedSquIDController(VelocityCompensatedSquIDTest.maxDecel, VelocityCompensatedSquIDTest.p, VelocityCompensatedSquIDTest.fric);

    public static double translationkP = 0.11;
    public static double translationkI = 0;
    public static double translationkD = 0;
    public static double headingkP = 0.002;
    public static double headingkI = 0;
    public static double headingkD = 0;

    public double tol = 3;
    public double hTol = 4;

    MecanumDriveSubsystem drive;
    PinpointSubsystem otos;

    Pose2d target;
    Pose2d currentPose;

    private DoubleSupplier xSpeedSupplier;
    private DoubleSupplier ySpeedSupplier;
    private DoubleSupplier rotSpeedSupplier;



    private boolean shouldLog = true;

    public DefaultGoToPointCommand(MecanumDriveSubsystem driveSubsystem, PinpointSubsystem otosSubsystem, Pose2d targetPose, double hTol){
        this(driveSubsystem, otosSubsystem, targetPose);
        this.hTol = hTol;
    }

    public DefaultGoToPointCommand(MecanumDriveSubsystem driveSubsystem, PinpointSubsystem otosSubsystem, Pose2d targetPose) {
        drive = driveSubsystem;
        otos = otosSubsystem;
        target = targetPose;


        addRequirements(drive);
    }

    @Override
    public void initialize(){
        xPID.setTolerance(tol);
        yPID.setTolerance(tol);
        headingPID.setTolerance(hTol);

        currentPose = otos.getPose();
        if (currentPose == null) {
            Log.i("execute", "currentPose was null (why?????)");
        }

        xPID.setPID(translationkP,translationkI,translationkD);
        yPID.setPID(translationkP,translationkI,translationkD);
        headingPID.setPID(headingkP,headingkI,headingkD);

        rotSpeedSupplier = () -> Util.signedSqrt(headingPID.calculate(currentPose.getRotation().getDegrees(), target.getRotation().getDegrees()));
    }

    @Override
    public void execute() {
        currentPose = otos.getPose();
        if (currentPose == null) {
            Log.w("execute", "currentPose was null (why?????)");
        } else {
            //Log.i("execute", "current pose was NOT null");
        }


        double xDist = target.getX() - currentPose.getX();
        double yDist = target.getY() - currentPose.getY();
        Log.v("GTPC", "xDist: " + xDist + ", yDist: " + yDist);
        double angle = Math.atan2(yDist,xDist);
        //double magnitude = Math.pow(Math.hypot(xDist,yDist),0.5);
        //Log.v("GTPC", "Magnitude: " + magnitude);
        //magnitude = magnitude*translationkP;
        double magnitude = positionSquID.calculate(0, Math.hypot(xDist, yDist));
        Log.v("GTPC", "Output: " + magnitude);
        double xMove = Math.cos(angle)*magnitude;
        double yMove = Math.sin(angle)*magnitude;
        Log.v("GTPC", "target: (" + getTargetX() + ", " + getTargetY() + ")");
        Log.v("GTPC", "attempted movement: (" + xMove + ", " + yMove + ")");

        drive.driveFieldCentric(-xMove, -yMove/*.12*/, -rotSpeedSupplier.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean wasInterrupted) {
        if (wasInterrupted) {
            drive.driveFieldCentric(0,0,0);
            Log.w("Commands", "gtpc was interrupted. This should never happen!");
        } else {
            Log.w("GTPC", "this ended without interruption somehow???");
        }
    }

    public boolean isDone() {
        if (target == null) {
            Log.i("isDone", "target was null");
            return false;
        }

        if (currentPose == null) {
            Log.i("isDone", "currentPose was null");
            return false;
        }

        return shouldLog && (currentPose.getTranslation().getDistance(target.getTranslation()) < tol) && (Util.inRange(target.getRotation().getDegrees(), currentPose.getRotation().getDegrees(), hTol));
    }

    public void setTarget(Pose2d target){
        this.target = target;
    }

    public double getTargetHeading() {
        return target.getRotation().getDegrees();
    }

    public double getTargetX() {
        return target.getX();
    }

    public double getTargetY() {
        return target.getY();
    }
    public Pose2d getCurrentPose() {
        return currentPose;
    }
    public Translation2d getTranslation() {
        return currentPose.getTranslation();
    }
    public void setTolerances(double tol, double Htol) {
        this.tol = tol;
        this.hTol = Htol;
    }
}

