package org.firstinspires.ftc.teamcode.code.hardware.statemachineproject.RobotUtil;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.code.hardware.statemachineproject.HelperClasses.Firefly;

public class RobotPosition {


    private static Firefly myRobot;

    public static double worldXPos = 0.0;
    public static double worldYPos = 0.0;
    public static double worldHeadingRad = 0.0;


    public static void initPose(Pose2d pose, Firefly robot) {
        worldXPos = pose.getX();
        worldYPos = pose.getY();
        worldHeadingRad = pose.getHeading();
        myRobot = robot;
    }
    public static void giveMePose(Pose2d pose) {
        worldXPos = pose.getX();
        worldYPos = pose.getY();
        worldHeadingRad = pose.getHeading();
    }

}
