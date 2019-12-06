package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil;

import org.firstinspires.ftc.teamcode.HelperClasses.ppProject.company.Range;

import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.*;
import static org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.RobotPosition.*;

import static java.lang.Math.*;

public class RobotMovement {

    private static turnStates ourTurnStates = turnStates.speed;
    private enum turnStates {
        speed,
        accurate
    }

    private static boolean distanceCompleted;
    private static boolean turnCompleted;
    public static boolean goToPositionCompleted = distanceCompleted && turnCompleted;


    public static void goToPosition(double targetX, double targetY, double point_angle, double movement_speed, double point_speed) {

        // init bools for movement (if we have to loop this then we know we havent reached targ position so init bool)
        distanceCompleted = false;
        turnCompleted = false;


        //get our distance away from the point

        double distanceToPoint = Math.sqrt(Math.pow(targetX-scaledWorldXPos,2) + Math.pow(targetY-scaledWorldYPos,2));

        double angleToPoint = Math.atan2(targetY-scaledWorldYPos,targetX-scaledWorldXPos);
        double deltaAngleToPoint = AngleWrap(angleToPoint-(scaledWorldHeadingRad-Math.toRadians(90)));
        //x and y components required to move toward the next point (with angle correction)
        double relative_x_to_point = Math.cos(deltaAngleToPoint) * distanceToPoint;
        double relative_y_to_point = Math.sin(deltaAngleToPoint) * distanceToPoint;

        double relative_abs_x = Math.abs(relative_x_to_point);
        double relative_abs_y = Math.abs(relative_y_to_point);


        //preserve the shape (ratios) of our intended movement direction but scale it by movement_speed
        double movement_x_power = (relative_x_to_point / (relative_abs_y+relative_abs_x)) * movement_speed;
        double movement_y_power = (relative_y_to_point / (relative_abs_y+relative_abs_x)) * movement_speed;

        //every movement has two states, the fast "gunning" section and the slow refining part. turn this var off when close to target

        double radToTarget = AngleWrap(point_angle - scaledWorldHeadingRad);
        double turnPower = 0;

        if(ourTurnStates == turnStates.speed) {
            if(Math.abs(radToTarget) > toDegrees(10)) {
                turnPower = radToTarget > 0 ? point_speed : -point_speed;
            }

            else {
                ourTurnStates = turnStates.accurate;
            }
        }

        if(ourTurnStates == turnStates.accurate) {
                turnPower = point_speed * radToTarget / toRadians(10);
                turnPower = Range.clip(turnPower, -point_speed, point_speed);
        }


        movementTurn = turnPower;
        movementX = movement_x_power;
        movementY = movement_y_power;




        // use this to check if are movements are complete
        // these are false anyways at the beginning of every loop
        distanceCompleted = true;
        turnCompleted = true;

    }



    public static void stopMovement() {
        movementX = 0;
        movementY = 0;
        movementTurn = 0;
    }
}