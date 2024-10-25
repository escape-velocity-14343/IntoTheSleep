package org.firstinspires.ftc.teamcode.lib;



import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AnalogEncoder {

    private AnalogInput sensor;
    private double offset;
    private double maxVoltage;
    private boolean inverted;
    private double maxAngle;

    {
        // default to 3.3
        maxVoltage = 3.3;

        // default to 360 degrees
        maxAngle = 360;

        // default to 0
        offset = 0;

        // default to false
        inverted = false;
    }

    public AnalogEncoder(String key, HardwareMap hMap){
        this.sensor = hMap.analogInput.get(key);
    }

    public void setInverted(boolean inverted){
        this.inverted = inverted;
    }

    /**
     * Sets offset of the encoder
     */
    public void setPositionOffset(double offset){
        this.offset = offset;
    }

    public void setMaxVoltage(double maxVoltage){
        this.maxVoltage = maxVoltage;
    }

    /**
     * Sets the maximum angle of the analog encoder. This means that 3.3V = max angle.
     * @param maxAngle Angle at 3.3V.
     */
    public void setMaxAngle(double maxAngle) {
        this.maxAngle = maxAngle;
    }

    /**
     * @return Degrees
     */
    public double getAngle(){
        return AngleUnit.normalizeDegrees((inverted ? -1 : 1) * (sensor.getVoltage() * maxAngle/maxVoltage) + offset);
    }
    /**
    * @return who would know
    */
    public double getRadians() {
        return Math.toRadians(getAngle());
    }


}
