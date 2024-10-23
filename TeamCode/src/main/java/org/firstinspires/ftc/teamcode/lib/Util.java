package org.firstinspires.ftc.teamcode.lib;

public class Util {
    public static boolean inRange(double a, double b, double thres) {
        return Math.abs(a-b)<thres;
    }

}
