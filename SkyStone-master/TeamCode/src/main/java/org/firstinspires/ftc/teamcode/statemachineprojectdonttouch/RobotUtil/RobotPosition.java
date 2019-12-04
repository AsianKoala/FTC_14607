package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses.Firefly;

import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.AngleWrap;


/**
 * this is used so that we can put the roadrunner localization into vars
 */
public class RobotPosition {

    private static Firefly myRobot;


    public static double scaledWorldXPos = 0.0;
    public static double scaledWorldYPos = 0.0;
    public static double scaledWorldHeadingRad = 0.0;
    public static double worldXPos = 0.0;
    public static double worldYPos = 0.0;
    public static double worldHeadingRad = 0.0;
    public static Pose2d worldPose = new Pose2d(0,0,0);


    public static void initPose(Pose2d pose, Firefly robot) {
        worldXPos = pose.getX();
        worldYPos = pose.getY();
        worldHeadingRad = pose.getHeading();
        worldPose = pose;


        scaledWorldYPos = worldXPos + 72;
        scaledWorldXPos = -worldYPos + 72;
        scaledWorldHeadingRad = Math.PI + AngleWrap(worldHeadingRad);

        myRobot = robot;
    }
    public static void giveMePose(Pose2d pose) {
        worldXPos = pose.getX();
        worldYPos = pose.getY();
        worldHeadingRad = pose.getHeading();
        worldPose = pose;


        scaledWorldYPos = worldXPos + 72;
        scaledWorldXPos = -worldYPos + 72;
        scaledWorldHeadingRad = Math.PI + AngleWrap(worldHeadingRad);
    }


    /**
     * we put this in here so that myDriveTrain controls motor powers while this gets pose values from
     * myDriveTrain and uses them
     * @param pose
     */
    public static void setPose(Pose2d pose) {
        myRobot.myDriveTrain.setPoseEstimate(pose);
        giveMePose(pose);
    }
}
