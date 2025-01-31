package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;

import java.util.function.Supplier;

public class GoToPointWithDefaultCommand extends CommandBase {
    private Pose2d target;
    private Supplier<Pose2d> targetSupplier;
    private boolean useTargetSupplier = false;
    private DefaultGoToPointCommand gtpc;

    /**
     * Sets the target point of the default go to point command
     * @param target The target to go to
     * @param gtpc
     */
    public GoToPointWithDefaultCommand(Pose2d target, DefaultGoToPointCommand gtpc){
        this.target = target;
        this.gtpc = gtpc;
    }
    public GoToPointWithDefaultCommand(Pose2d target, DefaultGoToPointCommand gtpc, double tol, double hTol) {
        this.target = target;
        this.gtpc = gtpc;
        gtpc.setTolerances(tol, hTol);
    }

    public GoToPointWithDefaultCommand(Supplier<Pose2d> targetSupplier, DefaultGoToPointCommand gtpc) {
        useTargetSupplier = true;
        this.targetSupplier = targetSupplier;
        this.gtpc = gtpc;
    }

    public GoToPointWithDefaultCommand(Supplier<Pose2d> targetSupplier, DefaultGoToPointCommand gtpc, double tol, double hTol) {
        useTargetSupplier = true;
        this.targetSupplier = targetSupplier;
        this.gtpc = gtpc;
        gtpc.setTolerances(tol, hTol);
    }

    public void initialize(){
        if (useTargetSupplier) {
            target = targetSupplier.get();
        }
        gtpc.setTarget(target);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean wasInterrupted) {
        Log.i("%1", "gtp finished " + target.getX() + " " + target.getY() + " " + target.getRotation().getDegrees());
    }

    @Override
    public boolean isFinished() {
        gtpc.setTolerances(3, 4);
        return gtpc.isDone();
    }
}
