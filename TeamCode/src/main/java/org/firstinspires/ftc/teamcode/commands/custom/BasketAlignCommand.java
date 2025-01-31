package org.firstinspires.ftc.teamcode.commands.custom;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.lib.DrivetrainSquIDController;
import org.firstinspires.ftc.teamcode.lib.SquIDController;
import org.firstinspires.ftc.teamcode.subsystems.BasketSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

import java.util.function.DoubleSupplier;

@Config
public class BasketAlignCommand extends CommandBase {
    public static double targetXDistance = 10;
    public static double targetYDistance = 10;
    public static double targetHeading = -3.1415 / 4.0;
    public static double translationkP = DefaultGoToPointCommand.translationkP;//0.008;
    public static double headingkP = 0.02;
    public static double xOffset = 0;
    public static double yOffset = 0;

    private final MecanumDriveSubsystem mecanumDrive;
    private final BasketSensorSubsystem basketSensor;
    private final PinpointSubsystem pinpoint;
    private final SquIDController headingController;
    private final DrivetrainSquIDController dtController;
    private final double threshold;

    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;

    public double error;

    public BasketAlignCommand(MecanumDriveSubsystem mecanumDrive, BasketSensorSubsystem basketSensor, PinpointSubsystem pinpoint, double threshold) {
        this.mecanumDrive = mecanumDrive;
        this.basketSensor = basketSensor;
        this.pinpoint = pinpoint;
        addRequirements(mecanumDrive, basketSensor);
        headingController = new SquIDController();
        dtController = new DrivetrainSquIDController();
        this.threshold = threshold;

        xSupplier = () -> 0.0;
        ySupplier = () -> 0.0;
    }

    /**
     * Creates a never ending align command for testing
     *
     * @param mecanumDriveSubsystem the drive subsystem
     * @param basketSensorSubsystem the sensor subsystem
     */
    public BasketAlignCommand(MecanumDriveSubsystem mecanumDriveSubsystem, BasketSensorSubsystem basketSensorSubsystem, PinpointSubsystem pinpoint) {
        this(mecanumDriveSubsystem, basketSensorSubsystem, pinpoint, -1.0);
    }

    @Override
    public boolean isFinished() {
        return threshold > 0.0 && error < threshold;
    }

    @Override
    public void execute() {
        Translation2d pose = new Translation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble()).rotateBy(new Rotation2d(-targetHeading));
        double x = targetXDistance + pose.getX();
        double y = targetYDistance + pose.getY();

        headingController.setPID(headingkP);
        dtController.setPID(translationkP);

        Pose2d p = dtController.calculate(
                AutoConstants.scorePos.plus(new Transform2d(new Translation2d(pose.getX(), pose.getY()), new Rotation2d())),
                pinpoint.getPose(),// new Pose2d(x, y, new Rotation2d()),
                //new Pose2d(basketSensor.getSensorRight(), basketSensor.getSensorLeft(), new Rotation2d()),
                pinpoint.getVelocity()
        );
        double dtFactor = 1.0;// Math.pow(Math.cos(targetHeading - pinpoint.getPose().getHeading()), 3.0);
        double voltage = mecanumDrive.getAutoVoltageMult();
        mecanumDrive.driveFieldCentric(
                -p.getX() * dtFactor * voltage,
                 -p.getY() * dtFactor * voltage,
                -headingController.calculate(targetHeading, pinpoint.getPose().getHeading()) * voltage
        );
        error = Math.hypot(y - basketSensor.getSensorLeft(), x - basketSensor.getSensorRight());
        Log.v("basket error", Double.toString(error));
    }

    public BasketAlignCommand withXySupplier(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;

        return this;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            if (Math.abs(pinpoint.getPose().getRotation().getDegrees() - AutoConstants.scorePos.getRotation().getDegrees()) < 2.5) {
                Pose2d newPose = new Pose2d(AutoConstants.scorePos.getX() - basketSensor.getSensorRight() + xOffset, AutoConstants.scorePos.getY() - basketSensor.getSensorLeft() + yOffset, new Rotation2d());
                double distance = AutoConstants.scorePos.minus(newPose).getTranslation().getNorm();
                if (distance < 10.0) {
                    pinpoint.setPosition(newPose.getX(), newPose.getY());
                }
            }
        }
    }

    public Command whenClose(double activationDistance, Command command) {
        return alongWith(new WaitUntilCommand(() -> error < activationDistance).andThen(command));
    }
}
