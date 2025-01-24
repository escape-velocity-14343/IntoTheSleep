package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.CachingVoltageSensor;
import org.firstinspires.ftc.teamcode.lib.Localizer;

public class MecanumDriveSubsystem extends SubsystemBase {
    DcMotor fr, fl, br, bl;
    Localizer odo;
    CachingVoltageSensor voltage;
    public MecanumDriveSubsystem(DcMotor fr, DcMotor fl, DcMotor br, DcMotor bl, Localizer localizer, CachingVoltageSensor voltage) {
        this.fr = fr;
        this.fl = fl;
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        this.br = br;
        this.bl = bl;
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        this.odo = localizer;
        this.voltage = voltage;
    }
    public MecanumDriveSubsystem(String fr, String fl, String br, String bl, HardwareMap hMap, Localizer localizer, CachingVoltageSensor voltage) {
        this(hMap.dcMotor.get(fr), hMap.dcMotor.get(fl), hMap.dcMotor.get(br), hMap.dcMotor.get(bl), localizer, voltage);
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
        if (!(Double.valueOf(frontLeftPower).isNaN() ||
                Double.valueOf(backLeftPower).isNaN() ||
                Double.valueOf(frontRightPower).isNaN() ||
                Double.valueOf(backRightPower).isNaN())) {


            fl.setPower(frontLeftPower);
            bl.setPower(backLeftPower);
            fr.setPower(frontRightPower);
            br.setPower(backRightPower);
        }

        //FtcDashboard.getInstance().getTelemetry().addData("fl", frontLeftPower);
        //FtcDashboard.getInstance().getTelemetry().addData("br", backRightPower);
    }
    public void driveFieldCentric(double x, double y, double rx) {
        driveFieldCentric(x, y, rx, odo.getPose().getRotation().getDegrees());
    }

    public double getAutoVoltageMult() {
        return this.voltage.getVoltageNormalized();
    }

}