package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;

public class LEDTest extends LinearOpMode {
    LEDSubsystem ledSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        ledSubsystem = new LEDSubsystem(hardwareMap);

        boolean variable = false;

        while (!isStopRequested()){
            ledSubsystem.lightSwitch(variable);
            variable = !variable;

            wait(1000);
        }
    }
}
