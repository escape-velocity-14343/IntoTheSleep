package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Localizer;

public class MecanumDriveSubsystem extends SubsystemBase {
    DcMotor fr, fl, br, bl;
    Localizer odo;
    public MecanumDriveSubsystem(DcMotor fr, DcMotor fl, DcMotor br, DcMotor bl, Localizer localizer) {
        this.fr = fr;
        this.fl = fl;
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        this.br = br;
        this.bl = bl;
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        this.odo = localizer;
    }
    public MecanumDriveSubsystem(String fr, String fl, String br, String bl, HardwareMap hMap, Localizer localizer) {
        this(hMap.dcMotor.get(fr), hMap.dcMotor.get(fl), hMap.dcMotor.get(br), hMap.dcMotor.get(bl), localizer);
    }

    /**
     * @param x positive drives forward
     * @param y positive drives left
     * @param rx positive turns clockwise
     * @param heading in degrees
     * */
    public void driveFieldCentric(double x, double y, double rx, double heading) {

        double headingRads = -Math.toRadians(heading);

        double rotX = y * Math.cos(headingRads) + x * Math.sin(headingRads);

        double rotY = y * Math.sin(headingRads) - x * Math.cos(headingRads);


        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        fl.setPower(frontLeftPower);
        bl.setPower(backLeftPower);
        fr.setPower(frontRightPower);
        br.setPower(backRightPower);

        FtcDashboard.getInstance().getTelemetry().addData("fl", frontLeftPower);
        FtcDashboard.getInstance().getTelemetry().addData("br", backRightPower);
    }
    public void driveFieldCentric(double x, double y, double rx) {
        driveFieldCentric(x, y, rx, odo.getPose().getRotation().getDegrees());
    }


}