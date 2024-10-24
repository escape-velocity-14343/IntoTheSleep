package org.firstinspires.ftc.teamcode.subsystems;

import android.util.ArrayMap;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.ArrayList;
import java.util.Map;
import java.util.Objects;

public class LEDSubsystem extends SubsystemBase {
    private ServoImplEx light;
    public LEDSubsystem(HardwareMap hMap) {
        light = (ServoImplEx) hMap.get(Servo.class, "light");
        light.setPosition(0);
        light.setPwmEnable();
    }

    /**
     * True for on, false for off
     * @param state
     */
    public void lightSwitch(boolean state){
        if (state) {
            light.setPwmEnable();
        }
        else {
            light.setPwmDisable();
        }
    }
}
