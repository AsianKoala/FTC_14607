package org.firstinspires.ftc.teamcode.HelperClasses;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import static java.lang.Math.*;

public  class GLOBALS {

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





    public static double AngleWrap(double angle) {
        while(angle < Math.PI) {
            angle += 2 * Math.PI;
        }

        while(angle > Math.PI) {
            angle -= 2 * Math.PI;
        }

        return angle;
    }


    public static double rrAngleWrap(double angle) {
        while(angle < 0) {
            angle += 2 * PI;
        }

        while(angle > 2 * PI) {
            angle -= 2 * PI;
        }

        return angle;
    }

    /**
     * get it
     * cause I
     *
     * red
     *
     * it
     *
     *
     *
     *
     *
     * ill lead myself out
     *
     *
     * @param pose blue pose
     * @return reddited (get it) pose
     */
    public static Pose2d redditPose(Pose2d pose) {
        return new Pose2d(pose.getX(), -1 * pose.getY(), rrAngleWrap(pose.getHeading() + PI));
    }

    /****************** ------ POSITIONING CONSTANTS ----------*****************/
    public static Pose2d blueFoundationStart = new Pose2d(48, 54, toRadians(90));
    public static Pose2d blueFoundation = new Pose2d(48, 24, toRadians(90));
    public static Pose2d blueNotSafePark = new Pose2d(0,54, toRadians(90));
    public static Pose2d blueSafePark = new Pose2d(0, 34, toRadians(0));

    // red pose for foundation

    public static Pose2d redFoundationStart = redditPose(blueFoundationStart);
    public static Pose2d redFoundation = redditPose(blueFoundation);
    public static Pose2d redNotSafePark = redditPose(blueNotSafePark);
    public static Pose2d redSafePark = redditPose(blueSafePark);


}
