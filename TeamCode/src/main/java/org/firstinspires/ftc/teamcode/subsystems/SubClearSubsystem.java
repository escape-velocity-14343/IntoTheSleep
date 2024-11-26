package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;

public class SubClearSubsystem extends SubsystemBase {

    private ServoImplEx subClear;
    private boolean isClearing = false;

    public SubClearSubsystem(HardwareMap hwm) {
        this.subClear = (ServoImplEx) hwm.servo.get("stick");
    }

    public void open() {
        subClear.setPosition(IntakeConstants.subClearPos);
        isClearing = true;
    }

    public void close() {
        subClear.setPosition(IntakeConstants.subClearRetractPos);
        isClearing = false;
    }

    public void toggle() {
        subClear.setPosition(!isClearing ? IntakeConstants.subClearPos : IntakeConstants.subClearRetractPos);
    }

    public void setPosition(double position) {
        subClear.setPosition(position);
    }
    public void stop() {
        subClear.setPwmDisable();
    }

}
