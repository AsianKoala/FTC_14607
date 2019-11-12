package org.firstinspires.ftc.teamcode.lastMinutescuffedPP.Movement;

import org.firstinspires.ftc.teamcode.lastMinutescuffedPP.Hardware.Robot;
import com.acmerobotics.roadrunner.control.PIDCoefficients;

public class MyPosition {

    public static Robot myRobot;


    public static double encoderFrontLeftLast = 0.0;
    public static double encoderBackLeftLast = 0.0;
    public static double encoderFrontRightLast = 0.0;
    public static double encoderBackRightLast = 0.0;
    public static double auxEncoder = 0.0;

    public static double worldXPosition = 0.0;
    public static double worldYPosition = 0.0;
    public static double worldAngle_rad = 0.0;
    public static double worldAngle = 0.0;


    public static double currPos_tL = 0.0;
    public static double currPos_tR = 0.0;
    public static double currPos_bL = 0.0;
    public static double currPos_bR = 0.0;
    public static double currPos_a = 0.0;


    // to read angle in absolute manner
    public static double frontLeftEncoderInitialReading = 0.0;
    public static double leftEncoderInitialReading = 0.0;
    public static double lastResetAngle = 0.0;


    public static double currentTravelYDistance = 0.0;


    public static void initialize(double tL, double bL, double tR, double bR, Robot myRobot) {
        MyPosition.myRobot = myRobot;
        currPos_tL = tL;
        currPos_bL = bL;
        currPos_tR = tR;
        currPos_bR = bR;

        update();
    }

    public static void giveMePositions(double tL, double bL, double tR, double bR, double wa) {
        currPos_tL = tL;
        currPos_bL = bL;
        currPos_tR = tR;
        currPos_bR = bR;
        worldAngle = wa;
        update();
    }

    public static void update() {
        PositioningCalculations();
    }


    /**
     * makes sure angle is in the range if -180 to 180
     * @param angle
     * @return
     */
    public static double AngleWrap(double angle){
        while (angle<-Math.PI){
            angle += 2.0*Math.PI;
        }
        while (angle>Math.PI){
            angle -= 2.0*Math.PI;
        }
        return angle;
    }

    public static double subtractAngles(double angle1, double angle2){
        return AngleWrap(angle1-angle2);
    }

    /**
     * Updates our position on the field using the change from the encoders
     */
    public static void PositioningCalculations(){
        double frontLeftCurrent = currPos_tL;
        double frontRightCurrent = currPos_tR;
        double backLeftCurrent = currPos_bL;
        double backRightCurrent = currPos_bR;


        // get deltas
        double frontLeftDelta = frontLeftCurrent - encoderFrontLeftLast;
        double frontRightDelta = frontRightCurrent - encoderFrontRightLast;
        double backLeftDelta = backLeftCurrent - encoderBackLeftLast;
        double backRightDelta = backRightCurrent - encoderBackRightLast;


        //get how much our angle has changed
        double worldAngleLast = worldAngle_rad;
        double angleIncrement = AngleWrap(worldAngle - worldAngleLast);


    }

}
