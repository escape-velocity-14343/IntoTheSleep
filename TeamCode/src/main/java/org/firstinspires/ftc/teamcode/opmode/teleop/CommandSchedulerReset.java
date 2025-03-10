package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group="0", name="Command Scheduler Reset")
public class CommandSchedulerReset extends LinearOpMode {

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();
    }

}
