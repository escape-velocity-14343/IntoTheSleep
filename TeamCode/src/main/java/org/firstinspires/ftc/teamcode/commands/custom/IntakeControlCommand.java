package org.firstinspires.ftc.teamcode.commands.custom;

import android.util.Log;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeControlCommand extends InstantCommand {
    IntakeSubsystem intake;
    double position = 0;
    double speed = 0;

    /**
     * Positive is outtake, negative is intake
     * @param intake
     * @param position
     */
    public IntakeControlCommand(IntakeSubsystem intake, double position, double speed) {
        addRequirements(intake);
        this.intake = intake;
        this.position = position;
        this.speed = speed;
    }
    @Override
    public void initialize() {
        intake.setClawer(position);
        intake.setIntakeSpeed(speed);
        Log.i("%8", "Intake pos: " + position);
    }
}
