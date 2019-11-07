/*package org.firstinspires.ftc.teamcode.lastMinutescuffedPP.Movement;

import org.firstinspires.ftc.teamcode.ppProject.company.ComputerDebugging;
import org.firstinspires.ftc.teamcode.lastMinutescuffedPP.HelperClasses.FloatPoint;
import org.firstinspires.ftc.teamcode.ppProject.company.Range; // TODO: FI ALL THESE DAMN IMPORTS REEE
import org.firstinspires.ftc.teamcode.ppProject.core.Point;
import org.firstinspires.ftc.teamcode.ppProject.treamcode.CurvePoint;
import org.firstinspires.ftc.teamcode.ppProject.treamcode.MathFunctions;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.lastMinutescuffedPP.Movement.MovementEssentials.smallAdjustSpeed;
import static org.firstinspires.ftc.teamcode.ppProject.RobotUtilities.MovementVars.*;
import static org.firstinspires.ftc.teamcode.ppProject.company.Robot.*;
import static org.firstinspires.ftc.teamcode.ppProject.treamcode.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.ppProject.treamcode.MathFunctions.lineCircleIntersection;

public class RobotMovement {
    public static profileStates state_movement_y_prof = profileStates.gunningIt;
    public static profileStates state_movement_x_prof = profileStates.gunningIt;
    public static profileStates state_turning_prof = profileStates.gunningIt;

    public static double movement_y_min = 0.091;
    public static double movement_x_min = 0.11;
    public static double movement_turn_min = 0.10;


    public enum profileStates{
        gunningIt,
        slipping,
        fineAdjustment,

        memes;

        private static profileStates[] vals = values();
        public profileStates next(){
            return vals[(this.ordinal()+1) % vals.length];
        }
    }

    //inits our mini state machines for motion profiling
    public static void initForMove() {
        state_movement_y_prof = profileStates.gunningIt;
        state_movement_x_prof = profileStates.gunningIt;
        state_turning_prof = profileStates.gunningIt;
    }




    
    // standard go to position using pure pursuit

    public static void goToPosition(double targetX, double targetY, double pointAngle, double movementSpeed, double pointSpeed) {
        double distanceToTarget = Math.hypot(targetX - worldXPosition, targetY - worldYPosition);
        
        double angleToPoint = Math.atan2(targetY - worldYPosition, targetX - worldYPosition);
        double deltaAngleToPoint = AngleWrap(angleToPoint-(worldAngle_rad-Math.toRadians(90)));
        
        // this has angle correction from anglewrap
        double relativeXToPoint = Math.cos(deltaAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(deltaAngleToPoint) * distanceToTarget;
        
        double relativeAbsX = Math.abs(relativeXToPoint);
        double relativeAbsY = Math.abs(relativeYToPoint);
        
        
        
        // preserve the shape (ratios) of our intended movement direction but scale with movement speed
       double movementXPower = (relativeXToPoint / (relativeAbsX + relativeAbsY)) * movementSpeed;
       double movementYPower = (relativeYToPoint / (relativeAbsX + relativeAbsY)) * movementSpeed;



        double rad_to_target = AngleWrap(pointAngle-worldAngle_rad);
        double turnPower = 0;

        //every movement has two states, the fast "gunning" section and the slow refining part. turn this var off when close to target
        if(state_turning_prof == profileStates.gunningIt) {
            turnPower = rad_to_target > 0 ? pointSpeed : -pointSpeed;
            if(Math.abs(rad_to_target) < Math.abs(SpeedOmeter.currSlipAngle() * 1.2) || Math.abs(rad_to_target) < Math.toRadians(3.0)){
                state_turning_prof = state_turning_prof.next();
            }

        }
        if(state_turning_prof == profileStates.slipping){
            if(Math.abs(SpeedOmeter.getDegPerSecond()) < 60){
                state_turning_prof = state_turning_prof.next();
            }

        }

        if(state_turning_prof == profileStates.fineAdjustment){
            //this is a var that will go from 0 to 1 in the course of 10 degrees from the target
            turnPower = (rad_to_target/Math.toRadians(10)) * smallAdjustSpeed;
            turnPower = Range.clip(turnPower,-smallAdjustSpeed,smallAdjustSpeed);
        }


        movement_turn = turnPower;
        movement_x = movementXPower;
        movement_y = movementYPower;
       
    }


    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius) {
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for(int i = 0; i < pathPoints.size() - 1; i++) {
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());



            double closestAngle = 100000;

            for (Point thisIntersection : intersections) {
                double angle = Math.atan2(thisIntersection.y - worldYPosition, thisIntersection.x - worldXPosition);
                double deltaAngle = Math.abs(MathFunctions.AngleWrap(angle - worldAngle_rad));

                if(deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
    }

    public static void followCurve  (ArrayList<CurvePoint> allPoints, double followAngle) {
        for(int i = 0; i < allPoints.size() - 1; i++) {
            ComputerDebugging.sendLine(new FloatPoint(allPoints.get(i).x, allPoints.get(i).y),
                    new FloatPoint(allPoints.get(i+1).x, allPoints.get(i+1).y));
        }

        CurvePoint followMe = getFollowPointPath(allPoints, new Point(worldXPosition, worldYPosition) ,
                allPoints.get(0).followDistance);

        ComputerDebugging.sendKeyPoint(new FloatPoint(followMe.x, followMe.y));

        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed); // important thing here
    }

}
*/