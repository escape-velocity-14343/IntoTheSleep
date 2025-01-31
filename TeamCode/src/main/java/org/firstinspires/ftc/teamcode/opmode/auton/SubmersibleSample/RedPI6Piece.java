package org.firstinspires.ftc.teamcode.opmode.auton.SubmersibleSample;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants.AutoConstants;

@Autonomous(name = "RED 7 Sample Popeyes' 7 Piece")
public class RedPI6Piece extends PI6Piece {

    @Override
    public void runOpMode() throws InterruptedException {
        AutoConstants.alliance = AutoConstants.Alliance.RED;
        super.runOpMode();
    }
}
