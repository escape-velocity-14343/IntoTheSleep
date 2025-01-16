package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

public class TurnToPointWithDefaultCommand extends CommandBase {
    private Pose2d target;
    private Pose2d currentPos;
    private DefaultGoToPointCommand gtpc;

    /**
     * Sets the target point of the default go to point command
     * @param target The target to go to
     * @param gtpc
     */
    public TurnToPointWithDefaultCommand(Pose2d target, DefaultGoToPointCommand gtpc){
        this.target = target;
        this.gtpc = gtpc;
    }
    public TurnToPointWithDefaultCommand(Pose2d target, DefaultGoToPointCommand gtpc, double tol, double hTol) {
        this.target = target;
        this.gtpc = gtpc;
        gtpc.setTolerances(tol, hTol);
    }

    public void initialize(){
        currentPos = gtpc.getCurrentPose();
        double dx = Math.abs(target.getX() - currentPos.getX());
        double dy = Math.abs(target.getY() - currentPos.getY());

        double angle = Math.atan(dy/dx);

        gtpc.setTarget(new Pose2d(currentPos.getX(), currentPos.getY(), Rotation2d.fromDegrees(angle)));
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean wasInterrupted) {
        Log.i("1", "gtp finished " + target.getX() + " " + target.getY() + " " + target.getRotation().getDegrees());
    }

    @Override
    public boolean isFinished() {
        gtpc.setTolerances(3, 4);
        return gtpc.isDone();
    }
}
