package org.firstinspires.ftc.teamcode.HelperClasses.ppProject.treamcode;

import org.firstinspires.ftc.teamcode.HelperClasses.ppProject.company.ComputerDebugging;
import org.firstinspires.ftc.teamcode.HelperClasses.ppProject.company.FloatPoint;
import org.firstinspires.ftc.teamcode.HelperClasses.ppProject.company.Range;
import org.firstinspires.ftc.teamcode.HelperClasses.ppProject.core.Point;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.HelperClasses.ppProject.RobotUtilities.MovementVars.*;
import static org.firstinspires.ftc.teamcode.HelperClasses.ppProject.company.Robot.*;
import static org.firstinspires.ftc.teamcode.HelperClasses.ppProject.company.Robot.worldXPosition;
import static org.firstinspires.ftc.teamcode.HelperClasses.ppProject.company.Robot.worldYPosition;
import static org.firstinspires.ftc.teamcode.HelperClasses.ppProject.treamcode.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.HelperClasses.ppProject.treamcode.MathFunctions.lineCircleIntersection;

public class RobotMovement {

    public static void followCurve (ArrayList<CurvePoint> allPoints, double followAngle) {
        for(int i = 0; i < allPoints.size() - 1; i++) {
            ComputerDebugging.sendLine(new FloatPoint(allPoints.get(i).x, allPoints.get(i).y),
                    new FloatPoint(allPoints.get(i+1).x, allPoints.get(i+1).y));
        }

        CurvePoint followMe = getFollowPointPath(allPoints, new Point(worldXPosition, worldYPosition) ,
                allPoints.get(0).followDistance);

        ComputerDebugging.sendKeyPoint(new FloatPoint(followMe.x, followMe.y));

        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed); // important thing here
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
                 double deltaAngle = Math.abs(AngleWrap(angle - worldAngle_rad));

                 if(deltaAngle < closestAngle) {
                     closestAngle = deltaAngle;
                     followMe.setPoint(thisIntersection);
                 }
             }
        }
        return followMe;
    }




    /**
     *
     *
     * @param x
     * @param y
     * @param movementSpeed
     * @param prefAngle
     * @param turnSpeed
     */
    public static void goToPosition(double x, double y, double movementSpeed, double prefAngle, double turnSpeed) {
        double distanceToTarget = Math.hypot(x-worldXPosition, y-worldYPosition);

        double absoluteAngleToTarget = Math.atan2(y-worldYPosition, x-worldXPosition);

        double relativeAngleToTarget = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));


        double relativeXToPoint = Math.cos(relativeAngleToTarget) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToTarget) * distanceToTarget;

        double v = Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint);
        double movementXPower = relativeXToPoint / v;
        double movementYPower = relativeYToPoint / v;

        movement_x = movementXPower * movementSpeed;
        movement_y = movementYPower * movementSpeed;

        double relativeTurnAngle = relativeAngleToTarget - Math.toRadians(180) + prefAngle;
        movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1) * turnSpeed;

        if(distanceToTarget < 10) {
            movement_turn = 0;
        }
    }
}
