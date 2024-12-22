package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BasketSensorSubsystem extends SubsystemBase {
    private final AnalogInput sensorLeft;
    private final AnalogInput sensorRight;

    public BasketSensorSubsystem(HardwareMap hardwareMap) {
        sensorLeft = hardwareMap.get(AnalogInput.class, "basketSensorLeft");
        sensorRight = hardwareMap.get(AnalogInput.class, "basketSensorRight");
    }

    public double getSensorLeft() {
        return sensorLeft.getVoltage()*500/3.3;
    }

    public double getSensorRight() {
        return sensorRight.getVoltage()*500/3.3;
    }
}
