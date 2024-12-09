package org.firstinspires.ftc.teamcode.commands.group;

import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.PivotConstants;
import org.firstinspires.ftc.teamcode.Constants.SlideConstants;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class RetractCommand extends SequentialCommandGroup {
    public RetractCommand(WristSubsystem wrist, PivotSubsystem pivot, ExtensionSubsystem extend){
        addCommands(new ParallelDeadlineGroup(new PivotCommand(pivot, PivotConstants.bottomLimit), new ExtendCommand(extend, SlideConstants.minExtension)), new WristCommand(wrist, IntakeConstants.foldedPos));
    }
}
