package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil;

import org.firstinspires.ftc.teamcode.HelperClasses.ppProject.company.Range;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses.WayPoint;


import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.*;
import static org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.RobotPosition.*;



import static java.lang.Math.*;

public class RobotMovement {

    private static final double smallAdjustSpeed = 0.135; // idk change this later ig ?


    private enum movementStates {
        iAmSpeed,
        accurate;

        private static movementStates[] vals = values();
        public movementStates next(){
            return vals[(this.ordinal()+1) % vals.length];
        }
    }

    private static movementStates turnStates = movementStates.iAmSpeed;
    private static movementStates xStates = movementStates.iAmSpeed;
    private static movementStates yStates = movementStates.iAmSpeed;

    public static void goToPosition(double targetX, double targetY, double point_angle, double movement_speed, double point_speed) {
        //get our distance away from the point
        double distanceToPoint = Math.sqrt(Math.pow(targetX-scaledWorldXPos,2) + Math.pow(targetY-scaledWorldYPos,2));

        double angleToPoint = Math.atan2(targetY-scaledWorldYPos,targetX-scaledWorldXPos);
        double deltaAngleToPoint = AngleWrap(angleToPoint-(worldHeadingRad-Math.toRadians(90)));
        //x and y components required to move toward the next point (with angle correction)
        double relative_x_to_point = Math.cos(deltaAngleToPoint) * distanceToPoint;
        double relative_y_to_point = Math.sin(deltaAngleToPoint) * distanceToPoint;

        double relative_abs_x = Math.abs(relative_x_to_point);
        double relative_abs_y = Math.abs(relative_y_to_point);




        //preserve the shape (ratios) of our intended movement direction but scale it by movement_speed
        double movement_x_power = (relative_x_to_point / (relative_abs_y+relative_abs_x)) * movement_speed;
        double movement_y_power = (relative_y_to_point / (relative_abs_y+relative_abs_x)) * movement_speed;

        //every movement has two states, the fast "gunning" section and the slow refining part. turn this var off when close to target
        if(yStates == movementStates.iAmSpeed) {
            if(relative_abs_y < 3) {
                yStates = yStates.next();
            }
        }
   
        if(yStates == movementStates.accurate){
            movement_y_power = Range.clip(((relative_y_to_point/8.0) * 0.15),-0.15,0.15);
        }

        if(xStates == movementStates.iAmSpeed) {
            if(relative_abs_x < 3) {
                xStates = xStates.next();
            }
        }

        if(xStates == movementStates.accurate){
            movement_x_power = Range.clip(((relative_x_to_point/2.5) * smallAdjustSpeed),-smallAdjustSpeed,smallAdjustSpeed);
        }

        double rad_to_target = AngleWrap(point_angle-scaledWorldHeadingRad);
        double turnPower = 0;

        //every movement has two states, the fast "gunning" section and the slow refining part. turn this var off when close to target
        if(turnStates == movementStates.iAmSpeed) {
            turnPower = rad_to_target > 0 ? point_speed : -point_speed;
            if(Math.abs(rad_to_target) < Math.toRadians(3.0)) {
                yStates = yStates.next();
            }
        }

        if(turnStates == movementStates.accurate){
            //this is a var that will go from 0 to 1 in the course of 10 degrees from the target
            turnPower = (rad_to_target/Math.toRadians(10)) * smallAdjustSpeed;
            turnPower = Range.clip(turnPower,-smallAdjustSpeed,smallAdjustSpeed);
        }

        movementTurn = turnPower;
        movementX = movement_x_power;
        movementY = movement_y_power;

        allComponentsMinPower();
    }





    // ok fine i stole this from gf BUT still its like only one thing dont get mad
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




    //point_angle is the relative point angle. 90 means face towards it
    public static void pointAngle(double point_angle, double point_speed, double decelerationRadians) {
        double relativePointAngle = AngleWrap(point_angle-scaledWorldHeadingRad);

        double turnSpeed = (relativePointAngle/decelerationRadians)*point_speed;
        movementTurn = Range.clip(turnSpeed,-point_speed,point_speed);

        allComponentsMinPower();

        // smooth as jello
        movementTurn *= Range.clip(Math.abs(relativePointAngle)/Math.toRadians(3),0,1);

    }





    public static boolean gunToPosition(double targetX, double targetY, double movementSpeed, double pointAngle, double pointSpeed, double slowDownTurnRad, double slowDownMovementFromTurnErrorMax, boolean stop ) {


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


        return currMovementStage > wayPoints.size() - 1;
    }



    public static void mecanumPower(double x, double y, double turn) {
        movementX = x;
        movementY = y;
        movementTurn = turn;
    }



    public static void stopMovement() {
        movementX = 0;
        movementY = 0;
        movementTurn = 0;
    }
}