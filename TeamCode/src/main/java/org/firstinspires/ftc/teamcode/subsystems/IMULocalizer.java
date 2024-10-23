package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.lib.Localizer;

public class IMULocalizer extends SubsystemBase implements Localizer {
    public IMULocalizer() {

    }
    @Override
    public Pose2d getPose() {
        return new Pose2d(0,0, new Rotation2d(0));
    }
}
