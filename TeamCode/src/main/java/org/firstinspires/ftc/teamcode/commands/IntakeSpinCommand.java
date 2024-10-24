package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeSpinCommand extends CommandBase {
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
