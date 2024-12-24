package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class BasketSensorSubsystem extends SubsystemBase {
    private final AnalogInput sensorLeft;
    private final AnalogInput sensorRight;
    private DistanceUnit unit = DistanceUnit.INCH;

    public BasketSensorSubsystem(HardwareMap hardwareMap) {
        sensorLeft = hardwareMap.get(AnalogInput.class, "basketSensorLeft");
        sensorRight = hardwareMap.get(AnalogInput.class, "basketSensorRight");
    }
    public void setDistanceUnit(DistanceUnit distanceUnit) {
        unit = distanceUnit;
    }
    /**
     * @return In whatever unit you set it to
     * */
    public double getSensorLeft() {
        return unit.fromCm(sensorLeft.getVoltage()*500/3.3);
    }
    /**
     * @return In whatever unit you set it to
     * */
    public double getSensorRight() {
        return unit.fromCm(sensorRight.getVoltage()*500/3.3);
    }
}
