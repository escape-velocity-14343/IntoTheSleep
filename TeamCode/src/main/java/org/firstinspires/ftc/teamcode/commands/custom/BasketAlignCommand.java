package org.firstinspires.ftc.teamcode.commands.custom;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.lib.SquIDController;
import org.firstinspires.ftc.teamcode.subsystems.BasketSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

import java.util.function.DoubleSupplier;

@Config
public class BasketAlignCommand extends CommandBase {
    public static double targetXDistance = 8;
    public static double targetYDistance = 8;
    public static double targetHeading = -3.1415 / 4.0;
    public static double translatekP = 0.008;
    public static double headingkP = 0.02;

    private final MecanumDriveSubsystem mecanumDrive;
    private final BasketSensorSubsystem basketSensor;
    private final PinpointSubsystem pinpoint;
    private final SquIDController headingController;
    private final SquIDController translationXController;
    private final SquIDController translationYController;
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
        translationXController = new SquIDController();
        translationYController = new SquIDController();
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

        // todo: once tuned, move this to the constructor or something idk
        headingController.setPID(headingkP);
        translationXController.setPID(translatekP);
        translationYController.setPID(translatekP);
        mecanumDrive.driveFieldCentric(
                -translationXController.calculate(x, basketSensor.getSensorRight() * Math.cos(targetHeading - pinpoint.getPose().getHeading())),
                translationYController.calculate(y, basketSensor.getSensorLeft() * Math.cos(targetHeading - pinpoint.getPose().getHeading())),
                -headingController.calculate(targetHeading, pinpoint.getPose().getHeading())
        );
        error = Math.hypot(y - basketSensor.getSensorLeft(), x - basketSensor.getSensorRight());
        Log.v("basket error", Double.toString(error));
    }

    public BasketAlignCommand withXySupplier(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;

        return this;
    }

    public Command whenClose(double activationDistance, Command command) {
        return alongWith(new WaitUntilCommand(() -> error < activationDistance).andThen(command));
    }
}
