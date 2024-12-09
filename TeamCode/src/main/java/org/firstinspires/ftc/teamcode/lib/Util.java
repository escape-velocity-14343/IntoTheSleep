package org.firstinspires.ftc.teamcode.lib;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

public class Util {
    public static boolean inRange(double a, double b, double thres) {
        return Math.abs(a-b)<thres;
    }

    /**
     * @param target The target angle, in degrees
     * @param current The current angle, in degrees
     * @return Returns the <i>shortest</i> angular difference between target and current
     */
    public static double getAngularDifference(double target, double current) {
        return posmod(target - current + 180, 360) - 180;
    }

    /**
     * @param x X
     * @param y Y
     * @return Returns the floating-point modulus of x divided by y, wrapping equally in positive and negative
     */
    public static double posmod(double x, double y) {
        return x - Math.floor(x / y) * y;
    }

    public static double clamp(double max, double min, double v){
        if (v > max){
            return max;
        }
        else if (v < min){
            return min;
        }
        return v;
    }
    public static double signedPower(double value, double power) {
        return Math.pow(Math.abs(value), power) * Math.signum(value);
    }
    public static double signedSqrt(double value) {
        return signedPower(value, 0.5);
    }
    public static Rotation2d angleToPoint(Pose2d current, Pose2d target) {
        Translation2d translation = target.getTranslation().minus(current.getTranslation());
        return new Rotation2d(Math.atan2(translation.getY(), translation.getX()));
    }
    public static Rotation2d angleToPoint(Translation2d current, Translation2d target) {
        Translation2d translation = target.minus(current);
        return new Rotation2d(Math.atan2(translation.getY(), translation.getX()));
    }

    public static double halfLinearHalfCubic(double input) {
        return (Math.pow(input, 3) + input) / 2;
    }

    public static double pose2dToDistance(Pose2d p1, Pose2d p2) {
        return Math.sqrt(Math.pow(p1.getX() - p2.getX(), 2) + Math.pow(p1.getY() - p2.getY(), 2));
    }
}
