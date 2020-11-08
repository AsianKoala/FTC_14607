package org.firstinspires.ftc.teamcode.util;

public class UtilMethods {

    public static double AngleWrap(double angle) {
        while(angle<-Math.PI) {
            angle += 2.0 * Math.PI;
        }
        while (angle>Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        return angle;
    }
}
