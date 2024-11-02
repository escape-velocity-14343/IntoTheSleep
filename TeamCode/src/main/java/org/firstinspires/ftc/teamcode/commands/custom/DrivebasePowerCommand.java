package org.firstinspires.ftc.teamcode.commands.custom;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class DrivebasePowerCommand extends CommandBase {
    private MecanumDriveSubsystem drive;
    private double xPower;
    private double yPower;
    private double hPower;

    public DrivebasePowerCommand(MecanumDriveSubsystem drive, double xPower, double yPower, double hPower) {
        this.drive = drive;
        this.xPower = xPower;
        this.yPower = yPower;
        this.hPower = hPower;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.driveFieldCentric(xPower, yPower, hPower);
    }
}
