package org.firstinspires.ftc.teamcode.commands.custom;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    MecanumDriveSubsystem drive;
    DoubleSupplier x, y, rx, heading;
    public DefaultDriveCommand(MecanumDriveSubsystem driveSubsystem, DoubleSupplier inputX, DoubleSupplier inputY, DoubleSupplier inputRx, DoubleSupplier robotHeading) {
        this.drive = driveSubsystem;
        this.x = inputX;
        this.y = inputY;
        this.rx = inputRx;
        this.heading = robotHeading;
        addRequirements(drive);
    }
    @Override
    public void execute() {
        drive.driveFieldCentric(-x.getAsDouble(),y.getAsDouble(),rx.getAsDouble(), heading.getAsDouble());
    }
}