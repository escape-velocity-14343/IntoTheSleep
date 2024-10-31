package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;

public class StayAtPointCommand extends CommandBase {
    private Pose2d target;
    private DefaultGoToPointCommand gtpc;

    public StayAtPointCommand(Pose2d target, DefaultGoToPointCommand gtpc){
        this.target = target;
        this.gtpc = gtpc;
    }

    public void initialize(){
        gtpc.setTarget(target);
    }

    @Override
    public void execute(){

    }

    @Override
    public void end(boolean wasInterrupted){
        Log.i("1", "gtp finished " + target.getX() + " " + target.getY() + " " + target.getRotation().getDegrees());
    }

    @Override
    public boolean isFinished(){
        return gtpc.isDone();
    }
}
