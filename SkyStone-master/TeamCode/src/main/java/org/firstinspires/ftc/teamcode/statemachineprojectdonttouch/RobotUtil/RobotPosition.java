package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses.Firefly;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses.OdometryModule;

import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.AngleWrap;


/**
 * this is used so that we can put the roadrunner localization into vars
 */
public class RobotPosition {

    public static double worldXPos, worldYPos, worldHeadingRad, scaledWorldYPos, scaledWorldXPos, scaledWorldHeadingRad;
    public static Pose2d worldPose;


    public static double leftEncoderMultipler, rightEncoderMultipler, middleEncoderMultiplier;


    private static double currLeftCount;
    private static double currRightCount;
    private static double currMiddleCount;
    private static double currHeading;

    private static double lastLeftCount;
    private static double lastMiddleCount;
    private static double lastRightCount;
    private static double lastHeading;




    private OdometryModule leftEncoder, rightEncoder, middleEncoder;

    // call this to update pose, input roadrunner pose
    public static void giveMePose(Pose2d pose) {
        worldXPos = pose.getX();
        worldYPos = pose.getY();
        worldHeadingRad = pose.getHeading();
        worldPose = pose;


        scaledWorldYPos = worldXPos + 72;
        scaledWorldXPos = -worldYPos + 72;
        scaledWorldHeadingRad = Math.PI + AngleWrap(worldHeadingRad);
    }

    public static void giveMePositions(double currLeft, double currMiddle, double currRight) {
        currLeftCount = currLeft;
        currMiddleCount = currMiddle;
        currRightCount = currRight;
    }



    public void update() {
        currLeftCount = leftEncoder.getReading() * leftEncoderMultipler;
        currRightCount = rightEncoder.getReading() * rightEncoderMultipler;


        double leftChange = currLeftCount - lastLeftCount;
        double rightChange = currRightCount - lastRightCount;

        // angle
        double deltaOrientation = (leftChange - rightChange /  )



        currMiddleCount = middleEncoder.getReading() * middleEncoderMultiplier;
        double rawHorizontalChange = currMiddleCount - lastMiddleCount;
        double horizontalChange = rawHorizontalChange;


        double p = (leftChange + rightChange)/2;
        double n = horizontalChange;


        worldXPos = worldXPos + (p * Math.sin(worldHeadingRad))


    }


    private void positionCalculations() {
         double scaledXPos = encoderTicksToInches((int)currLeftCount);
        double scaledYPos = encoderTicksToInches((int)currMiddleCount);




    }


    private static double encoderTicksToInches(int ticks) {
        return 2 * 2 * Math.PI  * ticks / TICKS_PER_REV;
    }
}
