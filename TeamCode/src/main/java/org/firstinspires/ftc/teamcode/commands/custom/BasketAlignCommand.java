package org.firstinspires.ftc.teamcode.commands.custom;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.lib.SquIDController;
import org.firstinspires.ftc.teamcode.subsystems.BasketSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

@Config
public class BasketAlignCommand extends CommandBase {
    public static double targetDistance = 20.0;
    public static double targetHeading = -3.1415 / 4.0;
    public static double translatekP = 0.005;
    public static double headingkP = 0.01;

    private final MecanumDriveSubsystem mecanumDrive;
    private final BasketSensorSubsystem basketSensor;
    private final PinpointSubsystem pinpoint;
    private final SquIDController headingController;
    private final SquIDController translationXController;
    private final SquIDController translationYController;
    private final double threshold;

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
    }

    /**
     * Creates a never ending align command for testing
     * @param mecanumDriveSubsystem the drive subsystem
     * @param basketSensorSubsystem the sensor subsystem
     */
    public BasketAlignCommand(MecanumDriveSubsystem mecanumDriveSubsystem, BasketSensorSubsystem basketSensorSubsystem, PinpointSubsystem pinpoint) {
        this(mecanumDriveSubsystem, basketSensorSubsystem, pinpoint, -1.0);
    }

    @Override
    public boolean isFinished() {
        return threshold > 0.0 && Math.abs(error) < threshold;
    }

    @Override
    public void execute() {
        // todo: once tuned, move this to the constructor or something idk
        headingController.setPID(headingkP);
        translationXController.setPID(translatekP);
        translationYController.setPID(translatekP);
        mecanumDrive.driveFieldCentric(
                -translationXController.calculate(targetDistance, basketSensor.getSensorRight()),
                translationYController.calculate(targetDistance, basketSensor.getSensorLeft()),
                -headingController.calculate(targetHeading, pinpoint.getPose().getHeading())
        );
    }
}
