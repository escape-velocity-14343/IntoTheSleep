package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;

public class SubClearSubsystem extends SubsystemBase {

    private ServoImplEx subClear, subClear2;
    private boolean isClearing = false;

    public SubClearSubsystem(HardwareMap hwm) {
        this.subClear = (ServoImplEx) hwm.servo.get("stick");
        this.subClear2 = (ServoImplEx) hwm.servo.get("stick2");
        close();
    }

    public void open() {
        subClear.setPosition(IntakeConstants.subClearLinearPos);
        subClear2.setPosition(IntakeConstants.subClear2LinearPos);
        isClearing = true;
    }

    public void wipe(boolean first) {
        if (first) {
            subClear.setPosition(IntakeConstants.subClearWipePos);
            subClear2.setPosition(IntakeConstants.subClear2RetractPos);
        } else {
            subClear.setPosition(IntakeConstants.subClearRetractPos);
            subClear2.setPosition(IntakeConstants.subClear2WipePos);
        }
        isClearing = true;
    }

    public void close() {
        subClear.setPosition(IntakeConstants.subClearRetractPos);
        subClear2.setPosition(IntakeConstants.subClear2RetractPos);
        isClearing = false;
    }

    public void setPosition(double position, double position2) {
        subClear.setPosition(position);
        subClear2.setPosition(position2);
    }
    public void stop() {
        subClear.setPwmDisable();
        subClear2.setPwmDisable();
    }

}
