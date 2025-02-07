package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.Constants.SlideConstants;
import org.firstinspires.ftc.teamcode.lib.AnalogEncoder;
import org.firstinspires.ftc.teamcode.lib.CachingVoltageSensor;
import org.firstinspires.ftc.teamcode.lib.SquIDController;
import org.firstinspires.ftc.teamcode.lib.Util;

public class PivotSubsystem extends SubsystemBase {
    private DcMotor motor0, motor1;
    private double currentPos = 0;
    private double pivotVelocity = 0;
    private ElapsedTime timer = new ElapsedTime();
    private double target = 0;
    private boolean manualControl = false;
    private SquIDController squid = new SquIDController();
    AnalogEncoder encoder;
    private CachingVoltageSensor voltage;

    public PivotSubsystem(HardwareMap hMap, CachingVoltageSensor voltage) {
        motor0 = hMap.dcMotor.get("tilt0");
        motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1 = hMap.dcMotor.get("tilt1");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder = new AnalogEncoder("sensOrange", hMap);
        encoder.setPositionOffset(PivotConstants.encoderOffset);
        encoder.setInverted(PivotConstants.encoderInvert);

        this.voltage = voltage;

        squid.setPID(PivotConstants.kP);
        timer.reset();
    }
    @Override
    public void periodic() {
        double lastPos = currentPos;
        currentPos = encoder.getAngle();
        pivotVelocity = (lastPos - currentPos) / timer.seconds();
        if (!manualControl) {
            tiltToPos(target);
        }
        timer.reset();
    }

    public void setPower(double power) {
        motor0.setPower(power*PivotConstants.direction);
        motor1.setPower(-power*PivotConstants.direction);
    }

    public void tiltToPos(double target) {
        manualControl = false;
        setTarget(target);
        double power = squid.calculate(target, getCurrentPosition()) * voltage.getVoltageNormalized();
        //if (currentPos > PivotConstants.topLimit-1 && power >= 0) {
        //    power = 0.3;
        //}
        if (power <= 0 && isClose(target) && target==PivotConstants.bottomLimit){
            power = -0.05;
        }
        setPower(power);
    }

    public void setTarget(double target){
        manualControl = false;
        this.target = target;
    }

    public double getPivotVelocity() {
        return pivotVelocity;
    }

    /**
    * @param target in inches, use the same one as the pid target
    */
    public boolean isClose(double target) {
        return Util.inRange(target, currentPos, PivotConstants.tolerance);// || currentPos < PivotConstants.bottomLimit;
    }

    public double getCurrentPosition() {
        return currentPos;
    }

    public void setManualControl(boolean manualControl) {
        this.manualControl = manualControl;
    }

    public boolean getManualControl() {
        return manualControl;
    }

    public void stop() {
        motor0.setPower(0);
        motor1.setPower(0);
    }
}
