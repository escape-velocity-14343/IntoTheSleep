package org.firstinspires.ftc.teamcode.commands.group;

import static org.firstinspires.ftc.teamcode.lib.Util.clamp;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OTOSSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

import java.util.function.DoubleSupplier;

@Config
public class GoToPointCommand extends CommandBase {
    public PIDController xPID = new PIDController(0,0,0);
    public PIDController yPID = new PIDController(0, 0,0);
    public PIDController headingPID = new PIDController(0,0,0);

    public static double translationkP = 0.15;
    public static double translationkI = 0;
    public static double translationkD = 0;
    public static double headingkP = 0.002;
    public static double headingkI = 0;
    public static double headingkD = 0;

    public double tol = 2;
    public double hTol = 2;

    MecanumDriveSubsystem drive;
    PinpointSubsystem otos;

    Pose2d target;
    Pose2d currentPose;

    private DoubleSupplier xSpeedSupplier;
    private DoubleSupplier ySpeedSupplier;
    private DoubleSupplier rotSpeedSupplier;

    public GoToPointCommand(MecanumDriveSubsystem driveSubsystem, PinpointSubsystem otosSubsystem, Pose2d targetPose, double hTol){
        this(driveSubsystem, otosSubsystem, targetPose);
        this.hTol = hTol;
    }

    public GoToPointCommand(MecanumDriveSubsystem driveSubsystem, PinpointSubsystem otosSubsystem, Pose2d targetPose) {
        drive = driveSubsystem;
        otos = otosSubsystem;
        target = targetPose;

        addRequirements(drive, otos);
    }

    @Override
    public void initialize() {
        xPID.setTolerance(tol);
        yPID.setTolerance(tol);
        headingPID.setTolerance(hTol);

        currentPose = otos.getPose();

        xPID.setPID(translationkP,translationkI,translationkD);
        yPID.setPID(translationkP,translationkI,translationkD);
        headingPID.setPID(headingkP,headingkI,headingkD);



        /*xSpeedSupplier = () -> xPID.calculate(currentPose.getX(), target.getX());
        ySpeedSupplier = () -> yPID.calculate(currentPose.getY(), target.getY());*/
        rotSpeedSupplier = () -> Util.signedSqrt(headingPID.calculate(currentPose.getRotation().getDegrees(), target.getRotation().getDegrees()));
    }

    @Override
    public void execute() {
        currentPose = otos.getPose();
        double xDist = target.getX() - currentPose.getX();
        double yDist = target.getY() - currentPose.getY();
        double angle = Math.atan2(yDist,xDist);
        double magnitude = Math.pow(Math.hypot(xDist,yDist),0.5);
        magnitude = xPID.calculate(0,magnitude);
        double xMove = Math.cos(angle)*magnitude;
        double yMove = Math.sin(angle)*magnitude;

        drive.driveFieldCentric(-xMove, -yMove, -rotSpeedSupplier.getAsDouble());
    }
    @Override
    public void end(boolean wasInterrupted) {
        drive.driveFieldCentric(0,0,0,0);
        Log.i("1", "gtp finished");
    }

    @Override
    public boolean isFinished() {
        return (currentPose.getTranslation().getDistance(target.getTranslation()) < tol) && (Util.inRange(target.getRotation().getDegrees(), currentPose.getRotation().getDegrees(), hTol));
    }

}

