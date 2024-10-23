package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SlideConstants;
import org.firstinspires.ftc.teamcode.lib.SquIDController;
import org.firstinspires.ftc.teamcode.lib.Util;

public class ExtensionSubsystem extends SubsystemBase {
    private DcMotor motor0, motor1;
    private int currentPos = 0;
    private PIDController pid = new PIDController(SlideConstants.kP, SlideConstants.kI, SlideConstants.kD);
    private SquIDController squid = new SquIDController();


    public ExtensionSubsystem(HardwareMap hMap) {
        motor0 = hMap.dcMotor.get("slide0");
        motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1 = hMap.dcMotor.get("slide1");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void periodic() {
        currentPos = -motor0.getCurrentPosition();
    }
    public void setPower(double power) {
        motor0.setPower(power*SlideConstants.direction);
        motor1.setPower(-power*SlideConstants.direction);
    }
    public void extendInches(double inches) {
        extendToPosition((int) (inches*SlideConstants.ticksPerInch));
    }
    public void extendToPosition(int ticks) {
        //pid.setPID(SlideConstants.kP, SlideConstants.kI, SlideConstants.kD);
        squid.setPID(SlideConstants.kP);
        //setPower(Math.sqrt(pid.calculate(getCurrentPosition(),ticks)));
        double power = squid.calculate(ticks, getCurrentPosition());
        if ((currentPos < 0 && power < 0)||currentPos > 27 && power > 0) {
            power = 0;
        }
        setPower(power);

    }
    /**
    * @param target in inches, use the same one as the pid target
    */
    public boolean isClose(double target) {
        return Util.inRange(target, getCurrentInches(), SlideConstants.tolerance);
    }


    public int getCurrentPosition() {
        return currentPos;
    }
    public double getCurrentInches() {
        return getCurrentPosition()/SlideConstants.ticksPerInch;
    }

    public void stop() {
        motor0.setPower(0);
        motor1.setPower(0);
    }
}
