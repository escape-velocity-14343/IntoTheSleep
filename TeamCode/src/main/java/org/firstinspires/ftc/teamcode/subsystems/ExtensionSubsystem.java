package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.SlideConstants;
import org.firstinspires.ftc.teamcode.lib.SquIDController;
import org.firstinspires.ftc.teamcode.lib.Util;

public class ExtensionSubsystem extends SubsystemBase {
    private DcMotor motor0, motor1;
    private int currentPos = 0;
    private PIDController pid = new PIDController(SlideConstants.kP, SlideConstants.kI, SlideConstants.kD);
    private SquIDController squid = new SquIDController();
    private double targetInches = 0;
    private boolean manualControl = true;


    public ExtensionSubsystem(HardwareMap hMap) {
        motor0 = hMap.dcMotor.get("slide0");
        motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1 = hMap.dcMotor.get("slide1");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void periodic() {
        currentPos = -motor0.getCurrentPosition();
        if (!manualControl)
            extendInches(targetInches);

    }


    /**
     *
     * this will also disable the pid until you turn it back on
     */
    public void setPower(double power) {
        motor0.setPower(power*SlideConstants.direction);
        motor1.setPower(-power*SlideConstants.direction);
    }
    public void setManualControl(boolean set) {
        this.manualControl = set;
    }

    /***
     * this sets the target as well
     */
    public void extendInches(double inches) {
        targetInches = inches;
        manualControl = false;
        extendToPosition((int) (inches*SlideConstants.ticksPerInch));
    }
    public void extendToPosition(int ticks) {
        //pid.setPID(SlideConstants.kP, SlideConstants.kI, SlideConstants.kD);
        squid.setPID(SlideConstants.kP);
        //setPower(Math.sqrt(pid.calculate(getCurrentPosition(),ticks)));

        double power = squid.calculate(ticks, getCurrentPosition());
        if (ticks>=0)
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
        setPower(0);
    }

}
