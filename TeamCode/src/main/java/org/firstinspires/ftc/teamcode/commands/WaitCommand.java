package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

public class WaitCommand extends CommandBase {
    ElapsedTime timer = new ElapsedTime();
    double seconds;

    /**
     * Seconds to wait
     * @param seconds
     */
    public WaitCommand(double seconds) {
        this.seconds = seconds;
        timer.reset();
    }

    @Override
    public boolean isFinished(){
        return (timer.milliseconds() * 1000 > seconds);
    }
}
