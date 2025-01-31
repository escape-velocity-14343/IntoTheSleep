package org.firstinspires.ftc.teamcode.commands.custom;

import android.util.Log;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeClawCommand extends InstantCommand {
    IntakeSubsystem intake;
    double position = 0;

    /**
     * Positive is outtake, negative is intake
     * @param intake
     * @param position
     */
    public IntakeClawCommand(IntakeSubsystem intake, double position) {
        addRequirements(intake);
        this.intake = intake;
        this.position = position;
    }
    @Override
    public void initialize() {
        intake.setClawer(position);
        Log.i("%8", "Intake pos: " + position);
    }
}
