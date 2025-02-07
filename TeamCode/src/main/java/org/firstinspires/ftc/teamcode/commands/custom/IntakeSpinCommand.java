package org.firstinspires.ftc.teamcode.commands.custom;

import android.util.Log;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeSpinCommand extends InstantCommand {
    IntakeSubsystem intake;
    double power = 0;

    /**
     * Positive is outtake, negative is intake
     * @param intake
     * @param power
     */
    public IntakeSpinCommand(IntakeSubsystem intake, double power) {
        addRequirements(intake);
        this.intake = intake;
        this.power = power;
    }
    @Override
    public void initialize() {
        intake.setIntakeSpeed(power);
        Log.i("%8", "Intake spin: " + power);
    }
}
