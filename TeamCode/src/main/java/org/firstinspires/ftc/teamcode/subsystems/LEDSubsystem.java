package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

public class LEDSubsystem extends SubsystemBase {
    private final Map<String, ServoImplEx> lights = new HashMap<>();

    public LEDSubsystem(HardwareMap hMap, String... lightNames) {
        for (String lightName : lightNames) {
            ServoImplEx light = hMap.get(ServoImplEx.class, lightName);
            lights.put(lightName, light);
            light.setPosition(0);
            light.setPwmEnable();
        }
    }

    /**
     * Turns on or off all the LEDS
     *
     * @param state if true, turns the lights on, otherwise turns them off
     */
    public void setLightState(boolean state) {
        for (ServoImplEx light : lights.values()) {
            if (state) {
                light.setPwmEnable();
            } else {
                light.setPwmDisable();
            }
        }
    }

    public void setLightState(boolean state, String ledName) {
        ServoImplEx light = Objects.requireNonNull(lights.get(ledName));
        if (state) {
            light.setPwmEnable();
        } else {
            light.setPwmDisable();
        }
    }
}
