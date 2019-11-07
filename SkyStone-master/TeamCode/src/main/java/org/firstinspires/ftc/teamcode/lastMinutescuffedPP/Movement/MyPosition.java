package org.firstinspires.ftc.teamcode.lastMinutescuffedPP.Movement;

import org.firstinspires.ftc.teamcode.lastMinutescuffedPP.Hardware.Robot;
import com.acmerobotics.roadrunner.control.PIDCoefficients;

public class MyPosition {

    public static Robot myRobot;


    public static double encoderLeftLast = 0.0;
    public static double encoderRightLast = 0.0;
    public static double auxEncoder = 0.0;

    public static double worldXPosition = 0.0;
    public static double worldYPosition = 0.0;
    public static double worldAngle_rad = 0.0;

    public static double currPos_l = 0.0;
    public static double currPos_r = 0.0;
    public static double currPos_a = 0.0;


    // to read angle in absolute manner
    public static double rightEncoderInitialReading = 0.0;
    public static double leftEncoderInitialReading = 0.0;
    public static double lastResetAngle = 0.0;


    public static double currentTravelYDistance = 0.0;


    public static void initialize(double l, double r, double a, Robot myRobot) {
        MyPosition.myRobot = myRobot;
        currPos_l = l;
        currPos_r = r;
        currPos_a = a;
        update();
    }

    public static void giveMePositions(double l, double r,double a) {
        currPos_l = l;
        currPos_r = r;
        currPos_a = a;
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
        double wheelLeftCurrent = -currPos_l;
        double wheelRightCurrent = currPos_r;
        double auxCurrent = currPos_a;


        // compute how much "wheel" data has changed
        double leftDelta = wheelLeftCurrent - encoderLeftLast;
        double rightDelta = wheelRightCurrent - encoderRightLast;
       // aux will be our horizontal movement,
        double auxDelta;


        // get distance traveled using the movement scaling



        double rightTotal = currPos_r - rightEncoderInitialReading;
        double leftTotal = -(currPos_l-leftEncoderInitialReading);

        double worldAngleLast = worldAngle_rad;


    }

}
