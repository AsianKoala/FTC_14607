package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses.OdometryModule;

import static org.firstinspires.ftc.teamcode.Auto.roadrunner.drive.DriveConstants.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.Auto.roadrunner.drive.DriveConstants.inchesToEncoderTicks;
import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.AngleWrap;
import static org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.RobotPosition.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.RobotPosition.worldYPos;


/**
 * this is used so that we can put the roadrunner localization into vars
 */
public class RobotPosition {

    private static double worldXPos, worldYPos, worldHeadingRad;
    public static double scaledWorldYPos, scaledWorldXPos, scaledWorldHeadingRad;
    public static Pose2d worldPose;






    private OdometryModule leftEncoder, rightEncoder, middleEncoder;

    // call this to update pose, input roadrunner pose
    public static void giveMePose(Pose2d pose) {
        worldXPos = pose.getX();
        worldYPos = pose.getY();
        worldHeadingRad = pose.getHeading();
        worldPose = pose;


        scaledWorldYPos = worldXPos + 72;
        scaledWorldXPos = -worldYPos + 72;
        scaledWorldHeadingRad = Math.PI/2 + AngleWrap(worldHeadingRad);
    }


    public static void giveMeScaledPos(Pose2d pose) {
        scaledWorldXPos = pose.getX();
        scaledWorldYPos = pose.getY();
        scaledWorldHeadingRad = pose.getHeading();
    }

    public static double encoderTicksToInches(int ticks) {
        return 2 * 2 * Math.PI  * ticks / TICKS_PER_REV;
    }




}


