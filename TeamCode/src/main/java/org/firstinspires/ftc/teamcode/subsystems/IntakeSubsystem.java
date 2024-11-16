package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;

@Config
public class IntakeSubsystem extends SubsystemBase {
    private CRServo intake;
    private Servo clawer;
    private double speed = 0;
    private double clawPos = IntakeConstants.closedPos;

    public IntakeSubsystem(HardwareMap hardwareMap){
        intake = hardwareMap.crservo.get("intake");
        clawer = hardwareMap.servo.get("clawer");
    }

    /**
     * Speeds from 1.0 to -1.0
     * Positive is outtake
     * Negative is intake
     * @param speed
     */
    public void setIntakeSpeed(double speed) {
        this.speed = speed;
        intake.setPower(speed);
    }
    public void setClawer(double value) {
        clawPos = value;
        clawer.setPosition(clawPos);
    }

    @Override
    public void periodic(){
        intake.setPower(-speed);
        clawer.setPosition(clawPos);
    }
}
