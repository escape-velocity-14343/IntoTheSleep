package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants.SlideConstants;
import org.firstinspires.ftc.teamcode.lib.SquIDController;
import org.firstinspires.ftc.teamcode.lib.Util;

public class ExtensionSubsystem extends SubsystemBase {
    private final DcMotorEx motor0;
    private final DcMotorEx motor1;
    private final PivotSubsystem pivotSubsystem;
    private int currentPos = 0;
    private final SquIDController squid = new SquIDController();
    private double targetInches = 0;
    private boolean manualControl = true;
    private int resetOffset = 0;

    private boolean speedToggle = false;

    private double extensionPowerMul = 1.0;

    public double getExtensionPowerMul() {
        return extensionPowerMul;
    }

    public void setExtensionPowerMul(double extensionPowerMul) {
        this.extensionPowerMul = extensionPowerMul;
    }

    public ExtensionSubsystem(HardwareMap hMap, PivotSubsystem pivotSubsystem) {
        this.pivotSubsystem = pivotSubsystem;

        motor0 = (DcMotorEx) hMap.dcMotor.get("slide0");
        motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1 = (DcMotorEx) hMap.dcMotor.get("slide1");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor0.setCurrentAlert(4, CurrentUnit.AMPS);
        motor1.setCurrentAlert(4, CurrentUnit.AMPS);
    }

    @Override
    public void periodic() {
        currentPos = -motor0.getCurrentPosition() - resetOffset;
        if (!manualControl) {
            extendInches(targetInches);
        }
    }


    /**
     *
     */
    public void setPower(double power) {

        if (speedToggle){
            power = 0.4;
        }
        motor0.setPower(power * SlideConstants.direction);
        motor1.setPower(-power * SlideConstants.direction);
        if (getCurrentPosition() < 10 && motor0.isOverCurrent() && motor1.isOverCurrent() && power < 0) {
            // resetOffset = getCurrentPosition();
        }
        FtcDashboard.getInstance().getTelemetry().addData("slide position", this.getCurrentInches());
        FtcDashboard.getInstance().getTelemetry().addData("slide motor power", power);
    }

    public void setManualControl(boolean set) {
        this.manualControl = set;
    }

    public boolean getManualControl() {
        return manualControl;
    }

    /**
     * this sets the target as well
     */
    public void extendInches(double inches) {
        targetInches = inches;
        manualControl = false;
        extendToPosition((int) (inches * SlideConstants.ticksPerInch));
    }

    public void extendToPosition(int ticks) {
        squid.setPID(SlideConstants.kP);

        // extensionPowerMul only applies to the squid output because the feedforward should stay constant
        double power =
                + squid.calculate(ticks, getCurrentPosition()) * extensionPowerMul
                + SlideConstants.FEEDFORWARD_STATIC
                + SlideConstants.FEEDFORWARD_DYNAMIC * Math.sin(pivotSubsystem.getCurrentPosition());

        if (ticks >= 0 && !(getCurrentInches() <= 0 && power < 0) && !(getCurrentInches() >= SlideConstants.maxExtension)) {
            setPower(power);
        }
        else{
            Log.i("A", "Extension limit has been breached");

        }
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
        return getCurrentPosition() / SlideConstants.ticksPerInch;
    }

    public void stop() {
        setPower(0);
    }

    public void reset() {
        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetOffset = 0;
    }

    public void setSpeedToggle(boolean b){
        speedToggle = b;
    }
}
