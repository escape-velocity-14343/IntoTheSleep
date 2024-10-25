package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.IntakePosCommand;

import org.firstinspires.ftc.teamcode.commands.IntakeEatCommand;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

@TeleOp
public class CommandBaseTest extends CommandOpMode {

    private PivotSubsystem pivotSubsystem;
    ExtensionSubsystem extend;
    WristSubsystem wrist;
    IntakeSubsystem intake;

    @Override
    public void initialize(){
        pivotSubsystem = new PivotSubsystem(hardwareMap);
        extend = new ExtensionSubsystem(hardwareMap);
        wrist = new WristSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);


        register(pivotSubsystem, extend, wrist, intake);


        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(new IntakePosCommand(extend, pivotSubsystem, wrist), new WaitCommand(1000), new IntakeEatCommand(extend,wrist,intake)));

    }
}
