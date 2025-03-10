package org.firstinspires.ftc.teamcode.commands.custom;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.BooleanSupplier;

public class InterruptCommand extends CommandBase {

    private BooleanSupplier interrupt;
    private Command command;

    public InterruptCommand(Command command, BooleanSupplier interrupt) {
        this.command = command;
        this.interrupt = interrupt;
    }

    @Override
    public void initialize() {
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return command.isFinished() || interrupt.getAsBoolean();
    }
}
