package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OTOSSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDrive extends CommandBase {
    MecanumDriveSubsystem drive;
    DoubleSupplier x, y, rx, heading;
    public DefaultDrive(MecanumDriveSubsystem driveSubsystem, DoubleSupplier inputX, DoubleSupplier inputY, DoubleSupplier inputRx) {
        this.drive = driveSubsystem;
        this.x = inputX;
        this.y = inputY;
        this.rx = inputRx;
        addRequirements(drive);
    }
    @Override
    public void execute() {
        drive.driveFieldCentric(-x.getAsDouble(),y.getAsDouble(),rx.getAsDouble(),0);
    }
}