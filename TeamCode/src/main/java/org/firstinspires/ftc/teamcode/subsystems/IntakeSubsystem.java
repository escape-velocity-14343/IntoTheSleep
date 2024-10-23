package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class IntakeSubsystem extends SubsystemBase {
    private CRServo intake;

    public IntakeSubsystem(HardwareMap hardwareMap){
        intake = hardwareMap.get(CRServo.class, "intake");
    }

    /**
     * Speeds from -1.0 to 1.0
     * Positive is outtake
     * Negative is intake
     * @param speed
     */
    public void setRotation(double speed){
        intake.setPower(speed);
    }
}
