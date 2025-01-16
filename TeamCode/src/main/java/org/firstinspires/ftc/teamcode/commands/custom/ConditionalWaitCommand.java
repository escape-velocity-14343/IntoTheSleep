package org.firstinspires.ftc.teamcode.commands.custom;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.qualcomm.robotcore.robocol.Command;

import java.util.function.Supplier;

public class ConditionalWaitCommand extends CommandBase {
    Supplier<Boolean> condition;

    public ConditionalWaitCommand(Supplier<Boolean> condition){
        this.condition = condition;
    }

    @Override
    public boolean isFinished() {
        return condition.get();
    }

    @Override
    public void end(boolean interrupted) {
        Log.i("10", "Conditional Wait Ended");
    }
}
