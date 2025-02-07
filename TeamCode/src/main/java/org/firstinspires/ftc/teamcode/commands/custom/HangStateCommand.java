package org.firstinspires.ftc.teamcode.commands.custom;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AscentSubsytem;

public class HangStateCommand extends CommandBase {
    private AscentSubsytem hanger;
    private AscentSubsytem.PTOMode mode;
    private ElapsedTime time = new ElapsedTime();
    public HangStateCommand(AscentSubsytem hang, AscentSubsytem.PTOMode mode) {
        hanger = hang;
        this.mode = mode;
    }

    @Override
    public void initialize() {
        hanger.setPto(mode);
        time.reset();
    }


    @Override
    public boolean isFinished() {
        return time.seconds()>0.25;
    }
}
