package org.firstinspires.ftc.teamcode.commands.custom;

import static org.firstinspires.ftc.teamcode.Constants.AutoConstants.autoscoreMaxVel;

import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

public class WaitUntilStabilized extends WaitUntilCommand {

    ElapsedTime timer = new ElapsedTime();
    Double timeout = 1.0;

    WaitUntilStabilized(PinpointSubsystem pinpointSubsystem){
        super(() -> pinpointSubsystem.getVelocity().getTranslation().getNorm() < autoscoreMaxVel);
        timer.reset();
        timer.startTime();
    }

    public void setTimeout(Double timeout){
        this.timeout = timeout;
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || timer.seconds() > timeout;
    }
}
