package org.firstinspires.ftc.teamcode.code.hardware.statemachineproject.RobotUtil;

import org.firstinspires.ftc.teamcode.code.hardware.statemachineproject.HelperClasses.Firefly;

public class RobotPosition {


    private static Firefly myRobot;

    public static double worldXPos = 0.0;
    public static double worldYPos = 0.0;
    public static double worldHeadingRad = 0.0;


    public static void initPositions(double worldX, double worldY, double worldHeading, Firefly robot) {
        worldXPos = worldX;
        worldYPos = worldY;
        worldHeadingRad = worldHeading;
        myRobot = robot;
    }
    public static void giveMePositions(double worldX, double worldY, double worldHeading) {
        worldXPos = worldX;
        worldYPos = worldY;
        worldHeadingRad = worldHeading;
    }

    public static void update() {

    }
}
