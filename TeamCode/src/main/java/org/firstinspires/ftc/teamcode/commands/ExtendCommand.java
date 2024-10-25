package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;

public class ExtendCommand extends CommandBase {
    ExtensionSubsystem extend;
    double target;

    /**
     *
     * @param subsystem
     * @param target in inches
     */
    public ExtendCommand(ExtensionSubsystem subsystem, double target) {
        this.extend = subsystem;
        this.target = target;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        extend.extendInches(target);
    }

    @Override
    public boolean isFinished(){
        return extend.isClose(target);
    }

}
