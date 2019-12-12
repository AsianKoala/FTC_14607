package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil;

import org.firstinspires.ftc.teamcode.HelperClasses.ppProject.company.Range;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses.WayPoint;


import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.*;
import static org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.RobotPosition.*;



import static java.lang.Math.*;

public class RobotMovement {

    private static turnStates ourTurnStates = turnStates.speed;
    private enum turnStates {
        speed,
        accurate
    }




    public static void mecanumPower(double x, double y, double turn) {
        movementX = x;
        movementY = y;
        movementTurn = turn;
    }





    public static void goToPosition(double targetX, double targetY, double point_angle, double movement_speed, double point_speed) {

        //get our distance away from the point

        double distanceToPoint = Math.sqrt(Math.pow(targetX-scaledWorldXPos,2) + Math.pow(targetY-scaledWorldYPos,2));

        double angleToPoint = Math.atan2(targetY-scaledWorldYPos,targetX-scaledWorldXPos);
        double deltaAngleToPoint = AngleWrap(angleToPoint-(scaledWorldHeadingRad));
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


    }

    public static void goToRRPosition(double targetX, double targetY, double point_angle, double movement_speed, double point_speed) {

        //get our distance away from the point

        double distanceToPoint = Math.sqrt(Math.pow(targetX-worldXPos,2) + Math.pow(targetY-worldYPos,2));

        double angleToPoint = Math.atan2(targetY-worldYPos,targetX-worldXPos);
        double deltaAngleToPoint = AngleWrap(angleToPoint-(scaledWorldHeadingRad));
        //x and y components required to move toward the next point (with angle correction)
        double relative_x_to_point = Math.cos(deltaAngleToPoint) * distanceToPoint;
        double relative_y_to_point = Math.sin(deltaAngleToPoint) * distanceToPoint;

        double relative_abs_x = Math.abs(relative_x_to_point);
        double relative_abs_y = Math.abs(relative_y_to_point);


        //preserve the shape (ratios) of our intended movement direction but scale it by movement_speed
        double movement_x_power = (relative_x_to_point / (relative_abs_y+relative_abs_x)) * movement_speed;
        double movement_y_power = (relative_y_to_point / (relative_abs_y+relative_abs_x)) * movement_speed;

        //every movement has two states, the fast "gunning" section and the slow refining part. turn this var off when close to target

        double radToTarget = AngleWrap(point_angle - worldHeadingRad);
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


    }


    public static void pointRRAngle(double pointAngle, double pointSpeed, double deccelRad) {
        double relativePointAngle = AngleWrap(pointAngle - worldHeadingRad);

        double turnSpeed = (relativePointAngle/deccelRad) * pointSpeed;

        movementTurn = Range.clip(turnSpeed, -pointSpeed, pointSpeed);
        movementTurn = Range.clip(Math.abs(relativePointAngle)/ Math.toRadians(3), 0,1);
    }


    public static void pointAngle(double pointAngle, double pointSpeed, double deccelRad) {
        double relativePointAngle = AngleWrap(pointAngle - scaledWorldHeadingRad);

        double turnSpeed = (relativePointAngle/deccelRad) * pointSpeed;

        movementTurn = Range.clip(turnSpeed, -pointSpeed, pointSpeed);
        movementTurn = Range.clip(Math.abs(relativePointAngle)/ Math.toRadians(3), 0,1);
    }



    public static boolean gunToPosition(double targetX, double targetY, double pointAngle, double movementSpeed, double pointSpeed, double slowDownTurnRad, double slowDownMovementFromTurnErrorMax, boolean stop ) {


        double distanceToTarget = Math.hypot(scaledWorldXPos - targetX, scaledWorldYPos - targetY);


        double angleToTarget = atan2(targetY - scaledWorldYPos, targetX - scaledWorldXPos);
        double relativeAngleToTarget = AngleWrap(angleToTarget - scaledWorldHeadingRad);


        // angle correction rel x and y
        double relativeXToPoint = Math.cos(relativeAngleToTarget) * distanceToTarget;
        double relativeYToPoint = sin(relativeAngleToTarget) * distanceToTarget;

        double relativeAbsX = Math.abs(relativeXToPoint);
        double relativeAbsY = Math.abs(relativeYToPoint);



        /* motor powers */

        //let's initialize to a power that doesn't care how far we are away from the point
        //We do this by just calculating the ratios (shape) of the movement with respect to
        //the sum of the two components, (sum of the absolute values to preserve the sines)
        //so total magnitude should always equal 1
        double movementXComponent = (relativeXToPoint / (relativeAbsX + relativeAbsY));
        double movementYComponent = (relativeYToPoint / (relativeAbsX + relativeAbsY));


        //So we will basically not care about what movementSpeed was given, we are going to
        //decelerate over the course of 30 cm anyways (100% to 0) and then clip the final values
        //to have a max of movement_speed.
        if (stop) {
            movementXComponent = relativeAbsX / 30.0;
            movementYComponent = relativeAbsY / 30.0;
        }

        movementX = Range.clip(movementXComponent, -movementSpeed, movementSpeed);
        movementY = Range.clip(movementYComponent, -movementSpeed, movementSpeed);


        // turn stuff here
        double absolutePointAngle = pointAngle + angleToTarget;


        double relativePointAngle = AngleWrap(absolutePointAngle - scaledWorldHeadingRad);


        // scale down angle by 40 and multiply by point
        double turnSpeed = (relativePointAngle / Math.toRadians(40)) * pointSpeed;


        movementTurn = Range.clip(turnSpeed, -pointSpeed, pointSpeed);
        if (distanceToTarget < 5) {
            movementTurn = 0;
        }


        // add smoothing near last 3 cm (no oscillate)
        movementX = Range.clip(relativeAbsX / 6, 0, 1);
        movementY = Range.clip(relativeAbsY / 6, 0, 1);

        movementTurn = Range.clip(Math.abs(relativePointAngle) / Math.toRadians(2), 0, 1);


        // slow down if heading off
        double turnErrorSoSlowDown = Range.clip(1.0 - Math.abs(relativePointAngle / slowDownTurnRad), 1.0 - slowDownMovementFromTurnErrorMax, 1);

        if (Math.abs(movementTurn) < 0.0001) {
            turnErrorSoSlowDown = 1;
        }


        movementX *= turnErrorSoSlowDown;
        movementY *= turnErrorSoSlowDown;


        return Math.abs(distanceToTarget) < 2 && Math.abs(relativeAngleToTarget) < Math.toDegrees(2);
    }



    private static int currMovementStage = 0;

    public static void initFollowCurve() {
        currMovementStage = 0;
    }


    public static boolean followCurve(ArrayList<WayPoint> wayPoints) {
        WayPoint targetWayPoint = wayPoints.get(currMovementStage);
        WayPoint endWayPoint = wayPoints.get(wayPoints.size() -1);
        boolean endResult;

        if(currMovementStage == wayPoints.size() -1) {
            if(gunToPosition(endWayPoint.targetX, endWayPoint.targetY, endWayPoint.movementSpeed, endWayPoint.pointAngle, endWayPoint.pointSpeed, endWayPoint.slowDownRadians, endWayPoint.slowDownErrorMax, true)) {
                currMovementStage = 1000000000;     // im pretty sure we wont have a path with these many waypoints but ok
            }
        }



        if(gunToPosition(targetWayPoint.targetX, targetWayPoint.targetY, targetWayPoint.movementSpeed, targetWayPoint.pointAngle, targetWayPoint.pointSpeed, targetWayPoint.slowDownRadians, targetWayPoint.slowDownErrorMax, false)) {
            currMovementStage++;
        }



        targetWayPoint = wayPoints.get(currMovementStage);

        return currMovementStage > wayPoints.size() - 1;
    }





    public static void gunToRRPosition(double targetX, double targetY,double point_angle,
                                               double movement_speed, double point_speed,
                                               double slowDownTurnRadians, double slowDownMovementFromTurnError,
                                               boolean stop) {



        //get our distance away from the adjusted point
        double distanceToPoint = Math.sqrt(Math.pow(worldXPos,2)
                + Math.pow(worldYPos,2));

        //arcTan gives the absolute angle from our location to the adjusted target
        double angleToPointAdjusted =
                Math.atan2(worldYPos, worldXPos);

        //we only care about the relative angle to the point, so subtract our angle
        //also subtract 90 since if we were 0 degrees (pointed at it) we use movement_y to
        //go forwards. This is a little bit counter-intuitive
        double deltaAngleToPointAdjusted = AngleWrap(angleToPointAdjusted-(worldHeadingRad-Math.toRadians(90)));

        //Relative x and y components required to move toward the next point (with angle correction)
        double relative_x_to_point = Math.cos(deltaAngleToPointAdjusted) * distanceToPoint;
        double relative_y_to_point = Math.sin(deltaAngleToPointAdjusted) * distanceToPoint;

        //just the absolute value of the relative components to the point (adjusted for slip)
        double relative_abs_x = Math.abs(relative_x_to_point);
        double relative_abs_y = Math.abs(relative_y_to_point);



        /**NOW WE CAN START CALCULATING THE POWER OF EACH MOTOR */
        //let's initialize to a power that doesn't care how far we are away from the point
        //We do this by just calculating the ratios (shape) of the movement with respect to
        //the sum of the two components, (sum of the absolute values to preserve the sines)
        //so total magnitude should always equal 1
        double movement_x_power = (relative_x_to_point / (relative_abs_y+relative_abs_x));
        double movement_y_power = (relative_y_to_point / (relative_abs_y+relative_abs_x));



        //So we will basically not care about what movement_speed was given, we are going to
        //decelerate over the course of 30 cm anyways (100% to 0) and then clip the final values
        //to have a max of movement_speed.
        if(stop){
            movement_x_power *= relative_abs_x / 30.0;
            movement_y_power *= relative_abs_y / 30.0;
        }


        //clip the final speed to be in the range the user wants
        movementX = Range.clip(movement_x_power,-movement_speed,movement_speed);
        movementY = Range.clip(movement_y_power,-movement_speed,movement_speed);







        /**NOW WE CAN DEAL WITH TURNING STUFF */
        //actualRelativePointAngle is adjusted for what side of the robot the user wants pointed
        //towards the point of course we need to subtract 90, since when the user says 90, we want
        //to be pointed straight at the point (relative angle of 0)
        double actualRelativePointAngle = (point_angle-Math.toRadians(90));

        //this is the absolute angle to the point on the field
        double angleToPointRaw = Math.atan2(targetY-worldYPos,targetX-worldXPos);
        //now if the point is 45 degrees away from us, we then add the actualRelativePointAngle
        //(0 if point_angle 90) to figure out the world angle we should point towards
        double absolutePointAngle = angleToPointRaw+actualRelativePointAngle;




        //now that we know what absolute angle to point to, we calculate how close we are to it
        double relativePointAngle = AngleWrap(absolutePointAngle-worldHeadingRad);


        //change the turn deceleration based on how fast we are going
        double decelerationDistance = Math.toRadians(40);


        //Scale down the relative angle by 40 and multiply by point speed
        double turnSpeed = (Math.toRadians(10)/decelerationDistance)*point_speed;





        //now just clip the result to be in range
        movementTurn = Range.clip(turnSpeed,-point_speed,point_speed);
        //HOWEVER don't go frantic when right next to the point
        if(distanceToPoint < 10){
            movementTurn = 0;
        }

        //make sure the largest component doesn't fall below it's minimum power
        allComponentsMinPower();


        //add a smoothing effect at the very last 3 cm, where we should turn everything off,
        //no oscillation around here
        movementX *= Range.clip((relative_abs_x/6.0),0,1);
        movementY *= Range.clip((relative_abs_y/6.0),0,1);

        movementTurn *= Range.clip(Math.abs(relativePointAngle)/Math.toRadians(2),0,1);


        //slow down if our point angle is off
        double errorTurnSoScaleDownMovement = Range.clip(1.0-Math.abs(relativePointAngle/slowDownTurnRadians),1.0-slowDownMovementFromTurnError,1);
        //don't slow down if we aren't trying to turn (distanceToPoint < 10)
        if(Math.abs(movementTurn) < 0.00001){
            errorTurnSoScaleDownMovement = 1;
        }
        movementX *= errorTurnSoScaleDownMovement;
        movementY *= errorTurnSoScaleDownMovement;


    }
    private static void allComponentsMinPower() {
        if(Math.abs(movementX) > Math.abs(movementY)){
            if(Math.abs(movementX) > Math.abs(movementTurn)){
                movementX = minPower(movementX,0.1);
            }else{
                movementTurn = minPower(movementTurn,0.1);
            }
        }else{
            if(Math.abs(movementY) > Math.abs(movementTurn)){
                movementY = minPower(movementY, 0.1);
            }else{
                movementTurn = minPower(movementTurn,0.1);
            }
        }
    }

    public static double minPower(double val, double min){
        if(val >= 0 && val <= min){
            return min;
        }
        if(val < 0 && val > -min){
            return -min;
        }
        return val;
    }

    public static void stopMovement() {
        movementX = 0;
        movementY = 0;
        movementTurn = 0;
    }
}