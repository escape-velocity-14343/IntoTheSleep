package org.firstinspires.ftc.teamcode.commands.group;

import static org.firstinspires.ftc.teamcode.lib.Util.clamp;

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

import java.util.function.DoubleSupplier;

@Config
public class GoToPointCommand extends CommandBase {
    public PIDController xPID = new PIDController(0,0,0);
    public PIDController yPID = new PIDController(0, 0,0);
    public PIDController headingPID = new PIDController(0,0,0);

    public static double translationkP = 0.02;
    public static double translationkI = 0;
    public static double translationkD = 0;
    public static double headingkP = 0.02;
    public static double headingkI = 0;
    public static double headingkD = 0;

    public static double tol = 2;
    public static double hTol = 2;

    MecanumDriveSubsystem drive;
    OTOSSubsystem otos;

    Pose2d target;
    Pose2d currentPose;

    private DoubleSupplier xSpeedSupplier;
    private DoubleSupplier ySpeedSupplier;
    private DoubleSupplier rotSpeedSupplier;

    public GoToPointCommand(MecanumDriveSubsystem driveSubsystem, OTOSSubsystem otosSubsystem, Pose2d targetPose) {
        drive = driveSubsystem;
        otos = otosSubsystem;
        target = targetPose;

        addRequirements(drive, otos);
    }

    @Override
    public void initialize(){
        xPID.setTolerance(tol);
        yPID.setTolerance(tol);
        headingPID.setTolerance(hTol);

        currentPose = otos.getPose();

        xPID.setPID(translationkP,translationkI,translationkD);
        yPID.setPID(translationkP,translationkI,translationkD);
        headingPID.setPID(headingkP,headingkI,headingkD);

        xSpeedSupplier = () -> xPID.calculate(currentPose.getX(), target.getX());
        ySpeedSupplier = () -> yPID.calculate(currentPose.getY(), target.getY());
        rotSpeedSupplier = () -> headingPID.calculate(currentPose.getHeading(), target.getHeading());
    }

    @Override
    public void execute() {
        currentPose = otos.getPose();

        drive.driveFieldCentric(clamp(1, -1.0, -xSpeedSupplier.getAsDouble()),
                clamp(1, -1.0, -ySpeedSupplier.getAsDouble()),
                clamp(1, -1.0, rotSpeedSupplier.getAsDouble()),
                0);
    }
    @Override
    public void end(boolean wasInterrupted) {
        drive.driveFieldCentric(0,0,0,0);
    }

    @Override
    public boolean isFinished() {
        FtcDashboard.getInstance().getTelemetry().addData("finished", xPID.atSetPoint() && yPID.atSetPoint() && headingPID.atSetPoint());

        return xPID.atSetPoint() && yPID.atSetPoint() && headingPID.atSetPoint();
    }
}

