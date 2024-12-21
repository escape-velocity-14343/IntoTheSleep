package org.firstinspires.ftc.teamcode.opmode.test;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.VelocityCompensatedSquIDController;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@TeleOp(group="1")
public class VelocityCompensatedSquIDTest extends Robot {

    public static double p = -0.00017;
    public static double maxDecel = 200;
    public static double fric = 0.003;

    public static double targetX = 0;
    public static double targetY = 0;

    public static double headingP = 0.002;

    @Override
    public void runOpMode() {
        initialize();

        VelocityCompensatedSquIDController velsquid = new VelocityCompensatedSquIDController(maxDecel, p, fric);

        waitForStart();

        pinpoint.reset();
        pinpoint.setPosition(0, 0);
        pinpoint.resetYaw();

        while (opModeIsActive()) {

            velsquid.setP(p);
            velsquid.setFrictionGain(fric);
            velsquid.setMaxDecel(maxDecel);
            pinpoint.periodic();

            double xDist = targetX - pinpoint.getPose().getX();
            double yDist = targetY - pinpoint.getPose().getY();
            Log.v("GTPC", "xDist: " + xDist + ", yDist: " + yDist);
            double angle = Math.atan2(yDist,xDist);
            double magnitude =velsquid.calculate(0, Math.hypot(xDist,yDist));
            Log.v("GTPC", "Magnitude: " + magnitude);
            double xMove = Math.cos(angle)*magnitude;
            double yMove = Math.sin(angle)*magnitude;
            Log.v("GTPC", "target: (" + targetX + ", " + targetY + ")");
            Log.v("GTPC", "attempted movement: (" + xMove + ", " + yMove + ")");
            telemetry.addData("TargetX", targetX);
            telemetry.addData("current X", pinpoint.getPose().getX());

            double headingError = 0 - pinpoint.getPose().getRotation().getDegrees();
            headingError = Math.signum(headingError) * headingP * Math.sqrt(Math.abs(headingError));

            mecanum.driveFieldCentric(-xMove, -yMove, headingError, pinpoint.getPose().getRotation().getDegrees());

            sleep(30);
            telemetry.update();
        }
    }

}
