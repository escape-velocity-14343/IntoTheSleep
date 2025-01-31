package org.firstinspires.ftc.teamcode.commands.custom;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.subsystems.AscentSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class L3HangRetract extends CommandBase {
    AscentSubsytem hang;
    ElapsedTime timer = new ElapsedTime();
    public L3HangRetract(AscentSubsytem hang, MecanumDriveSubsystem drive) {
        addRequirements(hang, drive);
        this.hang=hang;
    }
    public void initialize() {
        timer.reset();
    }

    public void execute() {
        hang.move(timer.seconds() > PivotConstants.l3hangtime ? -0.7 : -1);
    }
    public boolean isFinished() {
        return false;
    }
}
