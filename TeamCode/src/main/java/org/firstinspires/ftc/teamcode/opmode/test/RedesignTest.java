package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.lib.AnalogEncoder;

@TeleOp
@Config
public class RedesignTest extends LinearOpMode {
    public static double encoderOffset = 0;
    AnalogEncoder pivotEncoder;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor pivotMotorLeft = hardwareMap.get(DcMotor.class, "pivotLeft");
        DcMotor pivotMotorRight = hardwareMap.get(DcMotor.class, "pivotRight");
        pivotEncoder = new AnalogEncoder("pivotEncoder", hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (!isStopRequested()) {
            pivotEncoder.setPositionOffset(encoderOffset);
            double power = -gamepad1.left_stick_y;
            pivotMotorLeft.setPower(-power);
            pivotMotorRight.setPower(power);
            telemetry.addData("pivot encoder", pivotEncoder.getAngle());
            telemetry.addData("power", power);
            telemetry.update();
        }
    }
}
