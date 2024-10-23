package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;

@TeleOp
public class CommandBaseTest extends CommandOpMode {

    private PivotSubsystem pivotSubsystem;

    @Override
    public void initialize(){
        pivotSubsystem = new PivotSubsystem(hardwareMap);

        register(pivotSubsystem);


        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new WaitCommand(5.0),
                new PivotCommand(pivotSubsystem, 40.0)
        ));
    }
}
