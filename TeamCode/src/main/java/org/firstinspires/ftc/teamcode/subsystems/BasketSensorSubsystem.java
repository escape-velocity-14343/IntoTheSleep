package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.LinkedList;
import java.util.Queue;

@Config
public class BasketSensorSubsystem extends SubsystemBase {
    public static int rollingAverageSize = 3;

    private final AnalogInput sensorLeft;
    private final AnalogInput sensorRight;
    private DistanceUnit unit = DistanceUnit.INCH;

    private final Queue<Double> sensorLeftData = new LinkedList<>();
    private final Queue<Double> sensorRightData = new LinkedList<>();

    private double sensorLeftAverage;
    private double sensorRightAverage;

    public BasketSensorSubsystem(HardwareMap hardwareMap) {
        sensorLeft = hardwareMap.get(AnalogInput.class, "basketSensorLeft");
        sensorRight = hardwareMap.get(AnalogInput.class, "basketSensorRight");
    }

    @Override
    public void periodic() {
        sensorLeftData.add(sensorLeft.getVoltage());
        if (sensorLeftData.size() > rollingAverageSize) {
            sensorLeftData.remove();
        }

        // noinspection OptionalGetWithoutIsPresent
        sensorLeftAverage = sensorLeftData.stream()
                .reduce((total, el) -> total + el / sensorLeftData.size()).get();

        sensorRightData.add(sensorRight.getVoltage());
        if (sensorRightData.size() > rollingAverageSize) {
            sensorRightData.remove();
        }

        // noinspection OptionalGetWithoutIsPresent
        sensorRightAverage = sensorRightData.stream()
                .reduce((total, el) -> total + el / sensorRightData.size()).get();
    }

    public void setDistanceUnit(DistanceUnit distanceUnit) {
        unit = distanceUnit;
    }

    /**
     * @return In whatever unit you set it to
     */
    public double getSensorLeft() {
        return unit.fromCm(sensorLeftAverage * 500 / 3.3);
    }

    /**
     * @return In whatever unit you set it to
     */
    public double getSensorRight() {
        return unit.fromCm(sensorRightAverage * 500 / 3.3);
    }
}
