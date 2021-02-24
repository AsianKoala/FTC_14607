package org.firstinspires.ftc.teamcode.main.movement;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.main.control.Modifiers;
import org.firstinspires.ftc.teamcode.main.control.Results;
import org.firstinspires.ftc.teamcode.main.util.Point;
import org.firstinspires.ftc.teamcode.main.util.Util;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.main.hardware.DriveTrain.movementTurn;
import static org.firstinspires.ftc.teamcode.main.hardware.DriveTrain.movementX;
import static org.firstinspires.ftc.teamcode.main.hardware.DriveTrain.movementY;
import static org.firstinspires.ftc.teamcode.main.movement.Odometry.currentPosition;
import static org.firstinspires.ftc.teamcode.main.util.Util.angleWrap;
import static org.firstinspires.ftc.teamcode.main.util.Util.clipToFollowPointPath;
import static org.firstinspires.ftc.teamcode.main.util.Util.indexPoint;
import static org.firstinspires.ftc.teamcode.main.util.Util.lineCircleIntersection;
import static org.firstinspires.ftc.teamcode.main.util.Util.minCheck;


public class MovementController {


    public static Results.goToPositionResult goToPosition(double targetX, double targetY, double moveSpeed, double prefAngle, double turnSpeed, double slowDownTurnRadians, double slowDownMovementFromTurnError, double thresh, boolean stop) {
        double distance = Math.hypot(targetX - currentPosition.x, targetY - currentPosition.y);

        double absoluteAngleToTargetPoint = Math.atan2(targetY - currentPosition.y, targetX - currentPosition.x);
        double relativeAngleToTargetPoint = Util.angleWrap(absoluteAngleToTargetPoint - (currentPosition.heading - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToTargetPoint) * distance;
        double relativeYToPoint = Math.sin(relativeAngleToTargetPoint) * distance;
        double relativeAbsXToPoint = Math.abs(relativeXToPoint);
        double relativeAbsYToPoint = Math.abs(relativeYToPoint);

        double v = relativeAbsXToPoint + relativeAbsYToPoint;
        double movementXPower = relativeXToPoint / v;
        double movementYPower = relativeYToPoint / v;

        if (stop) {
            movementXPower *= relativeAbsXToPoint / 12;
            movementYPower *= relativeAbsYToPoint / 12;
        }

        movementX = Range.clip(movementXPower, -moveSpeed, moveSpeed);
        movementY = Range.clip(movementYPower, -moveSpeed, moveSpeed);


        // turning and smoothing shit
        double relativeTurnAngle = prefAngle - Math.toRadians(90);
        double absolutePointAngle = absoluteAngleToTargetPoint + relativeTurnAngle;
        double relativePointAngle = Util.angleWrap(absolutePointAngle - currentPosition.heading);

        double decelerateAngle = Math.toRadians(40);

        double movementTurnSpeed = (relativePointAngle / decelerateAngle) * turnSpeed;

        movementTurn = Range.clip(movementTurnSpeed, -turnSpeed, turnSpeed);

        if (distance < 3) {
            movementTurn = 0;
        }

        minCheck();


        // smoothing
        movementX *= Range.clip((relativeAbsXToPoint / 3.0), 0, 1);
        movementY *= Range.clip((relativeAbsYToPoint / 3.0), 0, 1);
        movementTurn *= Range.clip(Math.abs(relativePointAngle) / Math.toRadians(2), 0, 1);


        //slow down if our point angle is off
        double errorTurnSoScaleDownMovement = Range.clip(1.0 - Math.abs(relativePointAngle / slowDownTurnRadians), 1.0 - slowDownMovementFromTurnError, 1);
        //don't slow down if we aren't trying to turn (distanceToPoint < 3)
        if (Math.abs(movementTurn) < 0.00001) {
            errorTurnSoScaleDownMovement = 1;
        }
        movementX *= errorTurnSoScaleDownMovement;
        movementY *= errorTurnSoScaleDownMovement;

        return new Results.goToPositionResult(new Results.translationResult(targetX, targetY, thresh), new Results.turnResult(absolutePointAngle, Math.toRadians(2)));
    }


    public static Results.turnResult pointAngle(double point_angle, double point_speed, double decelerationRadians) {
        //now that we know what absolute angle to point to, we calculate how close we are to it
        double relativePointAngle = Util.angleWrap(point_angle - currentPosition.heading);

        //Scale down the relative angle by 40 and multiply by point speed
        double turnSpeed = (relativePointAngle / decelerationRadians) * point_speed;
        //now just clip the result to be in range
        movementTurn = Range.clip(turnSpeed, -point_speed, point_speed);

        //make sure the largest component doesn't fall below it's minimum power
        minCheck();

        //smooths down the last bit to finally settle on an angle
        movementTurn *= Range.clip(Math.abs(relativePointAngle) / Math.toRadians(3), 0, 1);

        return new Results.turnResult(point_angle, Math.toRadians(2));
    }


    public static Results.movementResult pointPointTurn(Point point, double point_speed, double decelerationRadians) {
        double absoluteAngleToTargetPoint = Math.atan2(point.y - currentPosition.y, point.x - currentPosition.x);
        return pointAngle(absoluteAngleToTargetPoint, point_speed, decelerationRadians);
    }


    public static Results.movementResult newFollowPath(PurePursuitPath path) {
        double baseFollowAngle = Math.toRadians(90); // base
        double baseMaxMoveSpeed = 0.8;
        double baseSlowDownTurnRadians = Math.toRadians(40);
        double baseSlowDownTurnAmount = 0.5;


        ArrayList<CurvePoint> allPoints = (ArrayList<CurvePoint>) path.allPoints.clone();
        //now we will extend the last line so that the pointing looks smooth at the end
        ArrayList<CurvePoint> pathExtended = (ArrayList<CurvePoint>) allPoints.clone();

        //first get which segment we are on
        indexPoint clippedToPath = clipToFollowPointPath(allPoints, currentPosition.x, currentPosition.y);
        int currFollowIndex = clippedToPath.index + 1;

        //get the point to follow
        CurvePoint followMe = getFollowPointPath(pathExtended, currentPosition.x, currentPosition.y,
                allPoints.get(currFollowIndex).followDistance);


        //this will change the last point to be extended

        CurvePoint firstExtendedPoint = allPoints.get(allPoints.size() - 2);
        CurvePoint secondExtendedPoint = allPoints.get(allPoints.size() - 1);
        double extendDistance = allPoints.get(allPoints.size() - 1).pointLength * 1.5;

        double lineAngle = Math.atan2(secondExtendedPoint.y - firstExtendedPoint.y, secondExtendedPoint.x - firstExtendedPoint.x);
        //get this line's length
        double lineLength = Math.hypot(secondExtendedPoint.x - firstExtendedPoint.x, secondExtendedPoint.y - firstExtendedPoint.y);
        //extend the line by 1.5 pointLengths so that we can still point to it when we
        //are at the end
        double extendedLineLength = lineLength + extendDistance;

        CurvePoint extended = new CurvePoint(secondExtendedPoint);
        extended.x = Math.cos(lineAngle) * extendedLineLength + firstExtendedPoint.x;
        extended.y = Math.sin(lineAngle) * extendedLineLength + firstExtendedPoint.y;

        pathExtended.set(pathExtended.size() - 1, extended);


        //get the point to point to
        CurvePoint pointToMe = getFollowPointPath(pathExtended, currentPosition.x, currentPosition.y,
                allPoints.get(currFollowIndex).pointLength);

//        followAngle = Math.atan2(0 - worldYPosition, 0 - worldXPosition);

        //if we are nearing the end (less than the follow dist amount to go) just manualControl point to end
        //but only if we have passed through the correct points beforehand
        double clipedDistToFinalEnd = Math.hypot(
                clippedToPath.point.x - allPoints.get(allPoints.size() - 1).x,
                clippedToPath.point.y - allPoints.get(allPoints.size() - 1).y);


        boolean superSmoother = false;
        if (clipedDistToFinalEnd <= followMe.followDistance + 6 ||
                Math.hypot(currentPosition.x - allPoints.get(allPoints.size() - 1).x,
                        currentPosition.y - allPoints.get(allPoints.size() - 1).y) < followMe.followDistance + 6) {
            superSmoother = true;
            followMe.setPoint(allPoints.get(allPoints.size() - 1).toPoint());
        }


        // HANDLE ALL TRANSLATION HERE
        double distance = Math.hypot(followMe.x - currentPosition.x, followMe.y - currentPosition.y);

        double absoluteAngleToTargetPoint = Math.atan2(followMe.y - currentPosition.y, followMe.x - currentPosition.x);
        double relativeAngleToTargetPoint = Util.angleWrap(absoluteAngleToTargetPoint - (currentPosition.heading - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToTargetPoint) * distance;
        double relativeYToPoint = Math.sin(relativeAngleToTargetPoint) * distance;
        double relativeAbsXToPoint = Math.abs(relativeXToPoint);
        double relativeAbsYToPoint = Math.abs(relativeYToPoint);

        double v = relativeAbsXToPoint + relativeAbsYToPoint;
        double movementXPower = relativeXToPoint / v;
        double movementYPower = relativeYToPoint / v;

        movementXPower *= relativeAbsXToPoint / 12;
        movementYPower *= relativeAbsYToPoint / 12;

        movementX = Range.clip(movementXPower, -baseMaxMoveSpeed, baseMaxMoveSpeed);
        movementY = Range.clip(movementYPower, -baseMaxMoveSpeed, baseMaxMoveSpeed);

        minCheck();




        // all turning shit here

        double currFollowAngle = Math.atan2(pointToMe.y - currentPosition.y, pointToMe.x - currentPosition.x);
        currFollowAngle += angleWrap(baseFollowAngle - Math.toRadians(90));

        Results.baseResult r;

        if(followMe.modifier.condition != Conditions.NONE) {

        } else {
            if(followMe.modifier instanceof Modifiers.turnModifier) {
                Modifiers.turnModifier tMod = (Modifiers.turnModifier) followMe.modifier;
                currFollowAngle = tMod.angle();
                r = tMod.turnMethod();
            } else if(followMe.modifier instanceof Modifiers.hardwareModifier) {
                Modifiers.hardwareModifier hMod = (Modifiers.hardwareModifier) followMe.modifier;
                r = hMod.hardwareMethod();
            } else {
                Results.baseResult trueResult = new Results.baseResult();
                trueResult.done = true;
                r = trueResult;
            }
        }



        double absDeltaAngle = Math.abs(Util.angleWrap(currFollowAngle - currentPosition.heading));

        // smoothing
        if (!superSmoother) {
            movementX *= Range.clip((relativeAbsXToPoint / 3.0), 0, 1);
            movementY *= Range.clip((relativeAbsYToPoint / 3.0), 0, 1);
        }

        movementX *= 1 - Range.clip(absDeltaAngle / baseSlowDownTurnRadians, 0, baseSlowDownTurnAmount);
        movementY *= 1 - Range.clip(absDeltaAngle / baseSlowDownTurnRadians, 0, baseSlowDownTurnAmount);

        if (superSmoother) {
            movementX *= Range.clip(Math.abs(followMe.x - currentPosition.x), 0.5, 1); // 0.787
            movementY *= Range.clip(Math.abs(followMe.y - currentPosition.y), 0.5, 1);
            movementTurn *= 10000000; // idk fix this later or something fucker
        }


        return new Results.goToPositionResult(new Results.translationResult(followMe.x, followMe.y, 2), new Results.turnResult(currFollowAngle, Math.toRadians(2)));
    }


    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, double xPos, double yPos, double followRadius) {
        indexPoint clipped = clipToFollowPointPath(pathPoints, xPos, yPos);
        int currIndex = clipped.index;

        CurvePoint followMe = new CurvePoint(pathPoints.get(currIndex + 1));
        //by default go to the follow point
        followMe.setPoint(new Point(clipped.point.x, clipped.point.y));


        for (int i = 0; i < pathPoints.size() - 1; i++) {
            CurvePoint startPoint = pathPoints.get(i);
            CurvePoint endPoint = pathPoints.get(i + 1);

            ArrayList<Point> intersections = lineCircleIntersection(xPos, yPos, followRadius, startPoint.x, startPoint.y, endPoint.x, endPoint.y);

            double closestDistance = 10000000; // placeholder num
            for (Point thisIntersection : intersections) {


                double dist = Math.hypot(thisIntersection.x - pathPoints.get(pathPoints.size() - 1).x,
                        thisIntersection.y - pathPoints.get(pathPoints.size() - 1).y);

                //follow if the distance to the last point is less than the closestDistance
                if (dist < closestDistance) {
                    closestDistance = dist;
                    followMe.setPoint(thisIntersection);//set the point to the intersection
                }
            }
        }

        return followMe;
    }

}
