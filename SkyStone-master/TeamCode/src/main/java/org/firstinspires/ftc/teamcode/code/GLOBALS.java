package org.firstinspires.ftc.teamcode.code;

public class GLOBALS {

    public final static double flipperHome =  0.95;
    public final static double flipperOut = 0.25;
    public final static double flipperBetween = (flipperHome + flipperOut)/2;
    public final static double flipperBetweenBetween = (flipperBetween + flipperOut)/2;
    public final static double rotaterHome = 0.279;
    public final static double rotaterOut = 0.95;
    public final static double gripperHome = 0.41;
    public final static double gripperGrip = 0.2;
    
    
    


    public final static long toMidTime = 450;
    public final static long liftTime = 200;
    public final static long toBackTime = 750;

    public final static long toLiftTimeTo = 400;
    public final static long toBackTimeTo = 700;

    public final static int liftIncrement = -200;
    public final static int liftIncrementer = -500;



    //ok these arent constants but stfu

    public static double movementX = 0;
    public static double movementY = 0;
    public static double movementTurn = 0;

    public static double frontLeftPower, frontRightPower, backLeftPower, backRightPower;



    public static double AngleWrap(double angle) {
        while(angle < Math.PI) {
            angle += 2 * Math.PI;
        }

        while(angle > Math.PI) {
            angle -= 2 * Math.PI;
        }

        return angle;
    }

}
