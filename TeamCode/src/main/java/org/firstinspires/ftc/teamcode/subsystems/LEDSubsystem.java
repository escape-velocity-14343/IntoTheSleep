package org.firstinspires.ftc.teamcode.subsystems;

import android.util.ArrayMap;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.Map;
import java.util.Objects;

import kotlin.jvm.functions.Function3;

public class LEDSubsystem extends SubsystemBase {
    // uses ArrayMap because HashMap seems overkill for 5 items
    ArrayMap<String, ServoImplEx> servos;

    public LEDSubsystem(HardwareMap hMap, String... entries) {
        for (String entry : entries) {
            servos.put(entry, hMap.get(ServoImplEx.class, entry));
        }
    }

    public void setLedState(String servoKey, boolean state) {
        if (state) {
            Objects.requireNonNull(servos.get(servoKey)).setPwmEnable();
        } else {
            Objects.requireNonNull(servos.get(servoKey)).setPwmDisable();
        }
    }

    /**
     * Sets multiple LED states at once, using ledPredicate to decide whether the LED is on or off
     * @param ledPredicate A function that accepts the index, the name, and the current state of each LED (servo)
     */
    public void setLedsState(Function3<Integer, String, Boolean, Boolean> ledPredicate) {
        // I put this method inside LEDSubsystem instead of LEDCommand to maintain strict access
        // to `servos`
        int i = 0;
        for (Map.Entry<String, ServoImplEx> led : servos.entrySet()) {
            if (ledPredicate.invoke(i++, led.getKey(), led.getValue().isPwmEnabled())) {
                led.getValue().setPwmEnable();
            } else {
                led.getValue().setPwmDisable();
            }
        }
    }
}
