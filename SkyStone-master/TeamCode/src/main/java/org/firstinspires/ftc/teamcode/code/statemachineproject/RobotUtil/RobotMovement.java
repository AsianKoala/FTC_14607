package org.firstinspires.ftc.teamcode.code.statemachineproject.RobotUtil;

import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.code.GLOBALS.*;
import static org.firstinspires.ftc.teamcode.code.statemachineproject.RobotUtil.RobotPosition.*;

/**
 * so i dont think this will work cause cause stupid roadrunner's localization IS SO FUCKED UP
 * like its coord system is like this
 *
 *  Q1  | Q4
 *  ---------
 *  Q2 | Q3
 */
@Deprecated
public class RobotMovement {


    /**
     *
     * @param x target x
     * @param y target y
     * @param movementSpeed movement speed
     * @param prefAngle preferred angle to target
     * @param turnSpeed turn speed
     */
    public static void goToPosition(double x, double y, double movementSpeed, double prefAngle, double turnSpeed, double allowableError) {
        double distanceToTarget = Math.hypot(x-worldXPos, y-worldYPos);

        double absoluteAngleToTarget = Math.atan2(y-worldYPos, x-worldXPos);

        double relativeAngleToTarget = AngleWrap(absoluteAngleToTarget - (worldHeadingRad - Math.toRadians(90)));


        double relativeXToPoint = Math.cos(relativeAngleToTarget) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToTarget) * distanceToTarget;

        double v = Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint);
        double movementXPower = relativeXToPoint / v;
        double movementYPower = relativeYToPoint / v;

        movementX = movementXPower * movementSpeed;
        movementY = movementYPower * movementSpeed;

        double relativeTurnAngle = relativeAngleToTarget - Math.toRadians(180) + prefAngle;
        movementTurn = Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1) * turnSpeed;

        if(distanceToTarget < allowableError) {
            movementX = 0;
            movementY = 0;
            movementTurn = 0;
        }

        if(distanceToTarget < 10) {
            movementTurn = 0;
        }
    }

    public double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle) * speed);
    }

    public double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle) * speed);
    }

}
