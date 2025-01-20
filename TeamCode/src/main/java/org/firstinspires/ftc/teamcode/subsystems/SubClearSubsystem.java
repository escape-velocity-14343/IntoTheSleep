package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;

public class SubClearSubsystem extends SubsystemBase {

    private ServoImplEx subClear;
    private ServoImplEx subClear2;
    private boolean isClearing = false;

    public SubClearSubsystem(HardwareMap hwm) {
        this.subClear = (ServoImplEx) hwm.servo.get("stick");
        this.subClear2 = (ServoImplEx) hwm.servo.get("stick2");

    }

    public void open() {
        subClear.setPosition(IntakeConstants.subClearPos);
        subClear2.setPosition(IntakeConstants.subClear2Pos);
        isClearing = true;
    }

    public void close() {
        subClear.setPosition(IntakeConstants.subClearRetractPos);
        subClear2.setPosition(IntakeConstants.subClear2RetractPos);
        isClearing = false;
    }

    public void toggle() {
        subClear.setPosition(!isClearing ? IntakeConstants.subClearPos : IntakeConstants.subClearRetractPos);
        subClear2.setPosition(isClearing ? IntakeConstants.subClear2Pos : IntakeConstants.subClear2RetractPos);
    }

    public void setPosition(double position) {
        subClear.setPosition(position);
        subClear2.setPosition(position);
    }

    public void dualSetPosition(double position1, double position2) {
        subClear.setPosition(position1);
        subClear2.setPosition(position2);
    }

    public void stop() {
        subClear.setPwmDisable();
        subClear2.setPwmDisable();
    }

}
