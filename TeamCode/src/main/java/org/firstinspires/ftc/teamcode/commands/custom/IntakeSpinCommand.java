package org.firstinspires.ftc.teamcode.commands.custom;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeSpinCommand extends InstantCommand {
    IntakeSubsystem intake;
    double power = 0;
    public IntakeSpinCommand(IntakeSubsystem intake, double power) {
        addRequirements(intake);
        this.intake = intake;
        this.power = power;
    }
    @Override
    public void initialize() {
        intake.setRotation(power);
    }
}
