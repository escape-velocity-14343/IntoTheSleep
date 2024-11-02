package org.firstinspires.ftc.teamcode.commands.custom;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;

import java.util.Objects;

import javax.annotation.Nullable;

public class ExtendCommand extends CommandBase {
    ExtensionSubsystem extend;
    double target;
    @Nullable
    private Double powerMul = null;
    private double oldExtensionPowerMul;

    /**
     * @param subsystem
     * @param target    in inches
     */
    public ExtendCommand(ExtensionSubsystem subsystem, double target) {
        this.extend = subsystem;
        this.target = target;
        addRequirements(subsystem);
    }

    /**
     * @param subsystem
     * @param target    in inches
     * @param powerMul  the power multipler to feed into the subsystem, will reset to the previous power multiplier once command finishes
     */
    public ExtendCommand(ExtensionSubsystem subsystem, double target, double powerMul) {
        this(subsystem, target);
        this.powerMul = powerMul;
    }

    @Override
    public void initialize() {
        if (powerMul != null) {
            oldExtensionPowerMul = extend.getExtensionPowerMul();
            extend.setExtensionPowerMul(powerMul);
        }
        extend.extendInches(target);
    }

    @Override
    public boolean isFinished() {
        return extend.isClose(target);
    }

    @Override
    public void end(boolean wasInterrupted) {
        if (powerMul != null) {
            extend.setExtensionPowerMul(oldExtensionPowerMul);
        }
        Log.i("9", "Extension to " + target);
    }
}
