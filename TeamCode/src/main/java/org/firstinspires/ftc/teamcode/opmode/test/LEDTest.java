package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
@TeleOp
public class LEDTest extends LinearOpMode {
    LEDSubsystem ledSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        DigitalChannel led1 = hardwareMap.digitalChannel.get("led1");
        led1.setMode(DigitalChannel.Mode.OUTPUT);
        DigitalChannel led2 = hardwareMap.digitalChannel.get("led2");
        led2.setMode(DigitalChannel.Mode.OUTPUT);
        DigitalChannel led3 = hardwareMap.digitalChannel.get("led3");
        led3.setMode(DigitalChannel.Mode.OUTPUT);
        DigitalChannel led4 = hardwareMap.digitalChannel.get("led4");
        led4.setMode(DigitalChannel.Mode.OUTPUT);
        while (!isStopRequested()) {
            led1.setState(gamepad1.a);
            led2.setState(gamepad1.b);
            led3.setState(gamepad1.x);
            led4.setState(gamepad1.y);
        }
    }
}
