package org.firstinspires.ftc.teamcode.control.controllers;

import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.util.MathUtil.*;

import static org.firstinspires.ftc.teamcode.hardware.DriveTrain.*;
import static org.firstinspires.ftc.teamcode.control.system.Robot.*;

import org.firstinspires.ftc.teamcode.util.CurvePoint;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.ArrayList;


public class OldPPController {

    public static double movement_y_min = 0.091;
    public static double movement_x_min = 0.11;
    public static double movement_turn_min = 0.10;

    private static void allComponentsMinPower() {
        if(Math.abs(powers.x) > Math.abs(powers.y)){
            if(Math.abs(powers.x) > Math.abs(powers.heading)){
                powers.x = minPower(powers.x,movement_x_min);
            }else{
                powers.heading = minPower(powers.heading,movement_turn_min);
            }
        }else{
            if(Math.abs(powers.y) > Math.abs(powers.heading)){
                powers.y = minPower(powers.y, movement_y_min);
            }else{
                powers.heading = minPower(powers.heading,movement_turn_min);
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



    public static class movementResult{
        public double turnDelta_rad;
        public movementResult(double turnDelta_rad){
            this.turnDelta_rad = turnDelta_rad;
        }
    }

    private static class indexPoint {
        int index;
        Point point;
        indexPoint(int index, Point point) {
            this.index = index;
            this.point = point;
        }
    }




    public static void goToPosition(double targetX, double targetY, double moveSpeed, double prefAngle, double turnSpeed, double slowDownTurnRadians, double slowDownMovementFromTurnError, boolean stop) {
        double distance = Math.hypot(targetX - currPose.x, targetY - currPose.y);

        double absoluteAngleToTargetPoint = Math.atan2(targetY - currPose.y, targetX - currPose.x);
        double relativeAngleToTargetPoint = MathUtil.angleWrap(absoluteAngleToTargetPoint - (currPose.heading - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToTargetPoint) * distance;
        double relativeYToPoint = Math.sin(relativeAngleToTargetPoint) * distance;
        double relativeAbsXToPoint = Math.abs(relativeXToPoint);
        double relativeAbsYToPoint = Math.abs(relativeYToPoint);

        double v = relativeAbsXToPoint + relativeAbsYToPoint;
        double movementXPower = relativeXToPoint / v;
        double movementYPower = relativeYToPoint / v;

        if(stop) {
            movementXPower *= relativeAbsXToPoint / 12;
            movementYPower *= relativeAbsYToPoint / 12;
        }

        powers.x = Range.clip(movementXPower, -moveSpeed, moveSpeed);
        powers.y = Range.clip(movementYPower, -moveSpeed, moveSpeed);



        // turning and smoothing shit
        double relativeTurnAngle = prefAngle - Math.toRadians(90);
        double absolutePointAngle = absoluteAngleToTargetPoint + relativeTurnAngle;
        double relativePointAngle = MathUtil.angleWrap(absolutePointAngle - currPose.heading);

        double decelerateAngle = Math.toRadians(40);

        double movementTurnSpeed = (relativePointAngle/decelerateAngle) * turnSpeed;

        powers.heading = Range.clip(movementTurnSpeed, -turnSpeed, turnSpeed);

        if(distance < 3) {
            powers.heading = 0;
        }

        allComponentsMinPower();


        // smoothing
        powers.x *= Range.clip((relativeAbsXToPoint/3.0),0,1);
        powers.y *= Range.clip((relativeAbsYToPoint/3.0),0,1);
        powers.heading *= Range.clip(Math.abs(relativePointAngle)/Math.toRadians(2),0,1);


        //slow down if our point angle is off
        double errorTurnSoScaleDownMovement = Range.clip(1.0-Math.abs(relativePointAngle/slowDownTurnRadians),1.0-slowDownMovementFromTurnError,1);
        //don't slow down if we aren't trying to turn (distanceToPoint < 10)
        if(Math.abs(powers.heading) < 0.00001){
            errorTurnSoScaleDownMovement = 1;
        }
        powers.x *= errorTurnSoScaleDownMovement;
        powers.y *= errorTurnSoScaleDownMovement;
    }


    public static movementResult pointAngle(double point_angle, double point_speed, double decelerationRadians) {
        //now that we know what absolute angle to point to, we calculate how close we are to it
        double relativePointAngle = MathUtil.angleWrap(point_angle-currPose.heading);

        //Scale down the relative angle by 40 and multiply by point speed
        double turnSpeed = (relativePointAngle/decelerationRadians)*point_speed;
        //now just clip the result to be in range
        powers.heading = Range.clip(turnSpeed,-point_speed,point_speed);

        //make sure the largest component doesn't fall below it's minimum power
        allComponentsMinPower();

        //smooths down the last bit to finally settle on an angle
        powers.heading *= Range.clip(Math.abs(relativePointAngle)/Math.toRadians(3),0,1);

        return new movementResult(relativePointAngle);
    }


    public static movementResult pointPointTurn(Point point, double point_speed, double decelerationRadians) {
        double absoluteAngleToTargetPoint = Math.atan2(point.y - currPose.y, point.x - currPose.x);
        return pointAngle(absoluteAngleToTargetPoint, point_speed, decelerationRadians);
    }


    public static boolean betterFollowCurve(ArrayList<CurvePoint> allPoints, double followAngle, Point anglePoint, boolean headingControlled){


        //now we will extend the last line so that the pointing looks smooth at the end
        ArrayList<CurvePoint> pathExtended = (ArrayList<CurvePoint>) allPoints.clone();

        //first get which segment we are on
        indexPoint clippedToPath = clipToFollowPointPath(allPoints,currPose.x,currPose.y);
        int currFollowIndex = clippedToPath.index+1;

        //get the point to follow
        CurvePoint followMe = getFollowPointPath(pathExtended,currPose.x,currPose.y,
                allPoints.get(currFollowIndex).followDistance);



        //this will change the last point to be extended
        pathExtended.set(pathExtended.size()-1,
                extendLine(allPoints.get(allPoints.size()-2),allPoints.get(allPoints.size()-1),
                        allPoints.get(allPoints.size()-1).pointLength * 1.5));



        //get the point to point to
        CurvePoint pointToMe = getFollowPointPath(pathExtended,currPose.x,currPose.y,
                allPoints.get(currFollowIndex).pointLength);

//        followAngle = Math.atan2(0 - worldYPosition, 0 - worldXPosition);

        //if we are nearing the end (less than the follow dist amount to go) just manualControl point to end
        //but only if we have passed through the correct points beforehand
        double clipedDistToFinalEnd = Math.hypot(
                clippedToPath.point.x-allPoints.get(allPoints.size()-1).x,
                clippedToPath.point.y-allPoints.get(allPoints.size()-1).y);


        boolean pepega = false;
        if(clipedDistToFinalEnd <= followMe.followDistance + 6 ||
                Math.hypot(currPose.x-allPoints.get(allPoints.size()-1).x,
                        currPose.y-allPoints.get(allPoints.size()-1).y) < followMe.followDistance + 6){
            pepega = true;
            followMe.setPoint(allPoints.get(allPoints.size()-1).toPoint());

        }




        goToPosition(followMe.x, followMe.y, followAngle,
                followMe.moveSpeed,followMe.turnSpeed,
                followMe.slowDownTurnRadians,0.2,true); // 0.275

        //find the angle to that point using atan2
        double currFollowAngle = Math.atan2(pointToMe.y-currPose.y,pointToMe.x-currPose.x);

        //if our follow angle is different, point differently
        currFollowAngle += angleWrap(followAngle - Math.toRadians(90));

        movementResult result;
        if(headingControlled) {
            result = pointPointTurn(anglePoint, allPoints.get(currFollowIndex).turnSpeed, Math.toRadians(45));
        } else {
            result = pointAngle(currFollowAngle,allPoints.get(currFollowIndex).turnSpeed,Math.toRadians(45));
        }

        powers.x *= 1 - Range.clip(Math.abs(result.turnDelta_rad) / followMe.slowDownTurnRadians,0,followMe.slowDownTurnAmount);
        powers.y *= 1 - Range.clip(Math.abs(result.turnDelta_rad) / followMe.slowDownTurnRadians,0,followMe.slowDownTurnAmount);

        if(pepega) {
            powers.x *= Range.clip(Math.abs(followMe.x-currPose.x)/0.8,0.5,1); // 0.787
            powers.y *= Range.clip(Math.abs(followMe.y-currPose.y)/0.8,0.5,1);

        }

        return clipedDistToFinalEnd < 4; //3
    }



    public static boolean followCurve(ArrayList<CurvePoint> allPoints, double followAngle){

        //now we will extend the last line so that the pointing looks smooth at the end
        ArrayList<CurvePoint> pathExtended = (ArrayList<CurvePoint>) allPoints.clone();

        //first get which segment we are on
        indexPoint clippedToPath = clipToFollowPointPath(allPoints,currPose.x,currPose.y);
        int currFollowIndex = clippedToPath.index+1;

        //get the point to follow
        CurvePoint followMe = getFollowPointPath(pathExtended,currPose.x,currPose.y,
                allPoints.get(currFollowIndex).followDistance);


        //this will change the last point to be extended
        pathExtended.set(pathExtended.size()-1,
                extendLine(allPoints.get(allPoints.size()-2),allPoints.get(allPoints.size()-1),
                        allPoints.get(allPoints.size()-1).pointLength * 1.5));



        //get the point to point to
        CurvePoint pointToMe = getFollowPointPath(pathExtended,currPose.x,currPose.y,
                allPoints.get(currFollowIndex).pointLength);



        //if we are nearing the end (less than the follow dist amount to go) just manualControl point to end
        //but only if we have passed through the correct points beforehand
        double clipedDistToFinalEnd = Math.hypot(
                clippedToPath.point.x-allPoints.get(allPoints.size()-1).x,
                clippedToPath.point.y-allPoints.get(allPoints.size()-1).y);



        if(clipedDistToFinalEnd <= followMe.followDistance + 6 ||
                Math.hypot(currPose.x-allPoints.get(allPoints.size()-1).x,
                        currPose.y-allPoints.get(allPoints.size()-1).y) < followMe.followDistance + 6){

            followMe.setPoint(allPoints.get(allPoints.size()-1).toPoint());
        }





        goToPosition(followMe.x, followMe.y,followAngle,
                followMe.moveSpeed,followMe.turnSpeed,
                followMe.slowDownTurnRadians,0,true);

        //find the angle to that point using atan2
        double currFollowAngle = Math.atan2(pointToMe.y-currPose.x,pointToMe.x-currPose.y);

        //if our follow angle is different, point differently
        currFollowAngle += MathUtil.angleWrap(followAngle - Math.toRadians(90));

        movementResult result = pointAngle(currFollowAngle,allPoints.get(currFollowIndex).turnSpeed,Math.toRadians(45));
        powers.x *= 1 - Range.clip(Math.abs(result.turnDelta_rad) / followMe.slowDownTurnRadians,0,followMe.slowDownTurnAmount);
        powers.y *= 1 - Range.clip(Math.abs(result.turnDelta_rad) / followMe.slowDownTurnRadians,0,followMe.slowDownTurnAmount);



        return clipedDistToFinalEnd < 4;// 4
    }





    /**
     * This will extend a line by a distance. It will modify only the second point
     */
    private static CurvePoint extendLine(CurvePoint firstPoint, CurvePoint secondPoint, double distance) {

        /*
         * Since we are pointing to this point, extend the line if it is the last line
         * but do nothing if it isn't the last line
         *
         * So if you imagine the robot is almost done its path, without this algorithm
         * it will just point to the last point on its path creating craziness around
         * the end (although this is covered by some sanity checks later).
         * With this, it will imagine the line extends further and point to a location
         * outside the endpoint of the line only if it's the last point. This makes the
         * last part a lot smoother, almost looking like a curve but not.
         */

        //get the angle of this line
        double lineAngle = Math.atan2(secondPoint.y - firstPoint.y,secondPoint.x - firstPoint.x);
        //get this line's length
        double lineLength = Math.hypot(secondPoint.x - firstPoint.x,secondPoint.y - firstPoint.y);
        //extend the line by 1.5 pointLengths so that we can still point to it when we
        //are at the end
        double extendedLineLength = lineLength + distance;

        CurvePoint extended = new CurvePoint(secondPoint);
        extended.x = Math.cos(lineAngle) * extendedLineLength + firstPoint.x;
        extended.y = Math.sin(lineAngle) * extendedLineLength + firstPoint.y;
        return extended;
    }



    // finds currPoint (startPoint of curr segment) on the current path
    public static indexPoint clipToFollowPointPath(ArrayList<CurvePoint> pathPoints, double xPos, double yPos) {
        double closestClip = 1000000000;

        // index of first point on line clipped
        int closestClippedIndex = 0;

        Point clipPoint = new Point();

        for(int i=0; i<pathPoints.size()-1; i++) {
            CurvePoint startPoint = pathPoints.get(i);
            CurvePoint endPoint = pathPoints.get(i+1);

            Point tempClipPoint = clipToLine(startPoint.x, startPoint.y, endPoint.x, endPoint.y, xPos, yPos);

            double distance = Math.hypot(xPos - tempClipPoint.x, yPos - tempClipPoint.y);

            if(distance < closestClip) {
                closestClip = distance;
                closestClippedIndex = i;
                clipPoint = tempClipPoint;
            }
        }
        return new indexPoint(closestClippedIndex, new Point(clipPoint.x, clipPoint.y));
    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, double xPos, double yPos, double followRadius) {
        indexPoint clipped = clipToFollowPointPath(pathPoints, xPos, yPos);
        int currIndex = clipped.index;

        CurvePoint followMe = new CurvePoint(pathPoints.get(currIndex+1));
        //by default go to the follow point
        followMe.setPoint(new Point(clipped.point.x,clipped.point.y));


        for(int i=0; i<pathPoints.size()-1; i++) {
            CurvePoint startPoint = pathPoints.get(i);
            CurvePoint endPoint = pathPoints.get(i+1);

            ArrayList<Point> intersections = lineCircleIntersection(xPos, yPos, followRadius, startPoint.x, startPoint.y, endPoint.x, endPoint.y);
//    public static Point circleLineIntersection(Point center, Point startPoint, Point endPoint, double radius) {
            double closestDistance = 10000000; // placeholder numb
            for(Point thisIntersection : intersections) {


                double dist = Math.hypot(thisIntersection.x - pathPoints.get(pathPoints.size()-1).x,
                        thisIntersection.y - pathPoints.get(pathPoints.size()-1).y);

                //follow if the distance to the last point is less than the closestDistance
                if(dist < closestDistance){
                    closestDistance = dist;
                    followMe.setPoint(thisIntersection);//set the point to the intersection
                }
            }
        }

        return followMe;
    }



    public static Point clipToLine(double lineX1, double lineY1, double lineX2, double lineY2,
                                   double robotX, double robotY){
        if(lineX1 == lineX2){
            lineX1 = lineX2 + 0.01;//nah
        }
        if(lineY1 == lineY2){
            lineY1 = lineY2 + 0.01;//nah
        }

        //calculate the slope of the line
        double m1 = (lineY2 - lineY1)/(lineX2 - lineX1);
        //calculate the slope perpendicular to this line
        double m2 = (lineX1 - lineX2)/(lineY2 - lineY1);

        //clip the robot's position to be on the line
        double xClipedToLine = ((-m2*robotX) + robotY + (m1 * lineX1) - lineY1)/(m1-m2);
        double yClipedToLine = (m1 * (xClipedToLine - lineX1)) + lineY1;
        return new Point(xClipedToLine,yClipedToLine);
    }


    public static ArrayList<Point> lineCircleIntersection(double circleX, double circleY, double r,
                                                          double lineX1, double lineY1,
                                                          double lineX2, double lineY2){
        //make sure the points don't exactly line up so the slopes work
        if(Math.abs(lineY1- lineY2) < 0.003){
            lineY1 = lineY2 + 0.003;
        }
        if(Math.abs(lineX1- lineX2) < 0.003){
            lineX1 = lineX2 + 0.003;
        }

        //calculate the slope of the line
        double m1 = (lineY2 - lineY1)/(lineX2-lineX1);

        //the first coefficient in the quadratic
        double quadraticA = 1.0 + pow(m1,2);

        //shift one of the line's points so it is relative to the circle
        double x1 = lineX1-circleX;
        double y1 = lineY1-circleY;


        //the second coefficient in the quadratic
        double quadraticB = (2.0 * m1 * y1) - (2.0 * pow(m1,2) * x1);

        //the third coefficient in the quadratic
        double quadraticC = ((pow(m1,2)*pow(x1,2)) - (2.0*y1*m1*x1) + pow(y1,2)-pow(r,2));


        ArrayList<Point> allPoints = new ArrayList<>();



        //this may give an error so we use a try catch
        try{
            //now solve the quadratic equation given the coefficients
            double xRoot1 = (-quadraticB + sqrt(pow(quadraticB,2) - (4.0 * quadraticA * quadraticC)))/(2.0*quadraticA);

            //we know the line equation so plug into that to get root
            double yRoot1 = m1 * (xRoot1 - x1) + y1;


            //now we can add back in translations
            xRoot1 += circleX;
            yRoot1 += circleY;

            //make sure it was within range of the segment
            double minX = Math.min(lineX1, lineX2);
            double maxX = Math.max(lineX1, lineX2);
            if(xRoot1 > minX && xRoot1 < maxX){
                allPoints.add(new Point(xRoot1,yRoot1));
            }

            //do the same for the other root
            double xRoot2 = (-quadraticB - sqrt(pow(quadraticB,2) - (4.0 * quadraticA * quadraticC)))/(2.0*quadraticA);

            double yRoot2 = m1 * (xRoot2 - x1) + y1;
            //now we can add back in translations
            xRoot2 += circleX;
            yRoot2 += circleY;

            //make sure it was within range of the segment
            if(xRoot2 > minX && xRoot2 < maxX){
                allPoints.add(new Point(xRoot2,yRoot2));
            }
        }catch(Exception e){
            //if there are no roots
        }
        return allPoints;
    }


}