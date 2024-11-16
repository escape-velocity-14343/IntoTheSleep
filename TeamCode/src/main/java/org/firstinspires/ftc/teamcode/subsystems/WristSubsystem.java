package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;

@Config
public class WristSubsystem extends SubsystemBase {
    private ServoImplEx wrist;
    private double rotation = IntakeConstants.groundPos;
    public static double debug = 0;

    public WristSubsystem(HardwareMap hardwareMap) {
        wrist = (ServoImplEx) hardwareMap.get(Servo.class, "wrist");
        wrist.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    @Override
    public void periodic(){
        wrist.setPosition(rotation);
    }

    public void setWrist(double rotation) {
        wrist.setPosition(rotation + IntakeConstants.wristOffset);
        this.rotation = rotation + IntakeConstants.wristOffset;
    }

    //TODO: if we use analog thing make it return actual position
    public double getPosition() {
        return wrist.getPosition();
    }
}
