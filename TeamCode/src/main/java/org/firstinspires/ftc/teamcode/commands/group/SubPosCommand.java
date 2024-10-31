package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeSpinCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class SubPosCommand extends SequentialCommandGroup {
    public SubPosCommand(ExtensionSubsystem extension, WristSubsystem wrist, IntakeSubsystem intake) {
        addCommands(new ExtendCommand(extension, 0), new WristCommand(wrist, IntakeConstants.groundPos).whenFinished(() -> Log.i("6", "Sub Pos Command")));
    }
}
