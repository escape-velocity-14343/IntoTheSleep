package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.DriveConstants;

public class AscentSubsytem extends SubsystemBase {

    private MecanumDriveSubsystem drive;
    private Servo ptoServoRight, ptoServoLeft;

    public enum PTOMode {
        ENGAGED,
        STOWED,
        FREEFLOAT
    }

    public AscentSubsytem(MecanumDriveSubsystem drive, HardwareMap hwmap) {
        this.drive = drive;
        this.ptoServoRight = hwmap.servo.get("pto0");
        this.ptoServoLeft = hwmap.servo.get("pto1");
        setPto(PTOMode.STOWED);
    }

    public void setPto(PTOMode mode) {
        switch (mode) {
            case ENGAGED:
                this.ptoServoLeft.setPosition(DriveConstants.ptoLeftEngagedPos);
                this.ptoServoRight.setPosition(DriveConstants.ptoRightEngagedPos);
                break;
            case STOWED:
                this.ptoServoLeft.setPosition(DriveConstants.ptoLeftDisengagedPos);
                this.ptoServoRight.setPosition(DriveConstants.ptoRightDisengagedPos);
                break;
            case FREEFLOAT:
                this.ptoServoLeft.setPosition(DriveConstants.ptoLeftFreeFloatPos);
                this.ptoServoRight.setPosition(DriveConstants.ptoRightFreeFloatPos);
                break;
        }
    }

    public void move(double power) {
        this.drive.bl.setPower(power);
        this.drive.br.setPower(power);
    }

    public void debugMove(double bl, double br) {
        this.drive.bl.setPower(bl);
        this.drive.br.setPower(br);
    }

}
