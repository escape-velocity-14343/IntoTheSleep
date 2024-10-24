package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeSpinCommand extends CommandBase {
    public IntakeSpinCommand(IntakeSubsystem intake, double power) {
        intake.setRotation(power);
        addRequirements(intake);
    }
}
