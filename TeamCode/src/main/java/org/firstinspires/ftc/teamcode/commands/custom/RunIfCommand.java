package org.firstinspires.ftc.teamcode.commands.custom;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;

import java.util.function.BooleanSupplier;

public class RunIfCommand extends ConditionalCommand {

    public RunIfCommand(Command command, BooleanSupplier booleanSupplier) {
        super(command, new InstantCommand(), booleanSupplier);
    }

}
