package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil;

import static org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.RobotPosition.encoderTicksToInches;

public class BetterRobotPosition {

    public static double worldXPosition, worldYPosition, worldAngleRad = 0;
    private static double currFL, currFR, currBL, currBR, currHeading = 0;

    private static double scale = encoderTicksToInches(1);

    private static double oldFL, oldFR, oldBL, oldBR = 0;

    public static void giveMeEncoders(double fl, double fr, double br, double bl, double heading) {
        currFL = fl;
        currFR = fr;
        currBR = br;
        currBL = bl;

        currHeading = heading;
    }

    public static void setPosition(double x, double y, double heading) {
        worldXPosition = x;
        worldYPosition = y;
        worldAngleRad = heading;
    }



    public static void dumbfuck() {
        double initialFL = currFL;
        double initialFR = currFR;
        double initialBR = currBR;
        double initialBL = currBL;



        double deltaScaledFL = (initialFL - oldFL) * scale;
        double deltaScaledFR = (initialFR - oldFR) * scale;
        double deltaScaledBR = (initialBR - oldBR) * scale;
        double deltaScaledBL = (initialBL - oldBL) * scale;



        worldAngleRad = currHeading;

        double deltaYTraveled = (deltaScaledBL + deltaScaledBR + deltaScaledFL + deltaScaledFR)/4;
        double deltaXTraveled = deltaScaledFL + deltaScaledBR - deltaScaledBL - deltaScaledFR;


        worldXPosition += (Math.cos(worldAngleRad) * deltaYTraveled) + (Math.sin(worldAngleRad) * deltaXTraveled);
        worldYPosition += (Math.sin(worldAngleRad) * deltaYTraveled) - (Math.cos(worldAngleRad) * deltaXTraveled);


        oldFL = initialFL;
        oldFR = initialFR;
        oldBL = initialBL;
        oldBR = initialBR;
    }

}
