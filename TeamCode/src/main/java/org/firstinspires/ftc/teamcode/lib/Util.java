package org.firstinspires.ftc.teamcode.lib;

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
}
