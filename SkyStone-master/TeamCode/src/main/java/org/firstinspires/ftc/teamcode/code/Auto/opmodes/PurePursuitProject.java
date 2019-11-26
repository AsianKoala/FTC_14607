package org.firstinspires.ftc.teamcode.code.Auto.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.code.Firefly;
import org.firstinspires.ftc.teamcode.code.Auto.opmodes.Point;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.ppProject.treamcode.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.code.Auto.opmodes.MathFunctions.*;

public class PurePursuitProject {

    private Firefly robot;

    public PurePursuitProject(Firefly robot) {
        this.robot = robot;
    }

    public void goToPosition(Pose2d targetPose, double movementSpeed, double turnSpeed, double allowableDistanceError, double nonAllowableTurnDist){
        double distanceToTarget = Math.hypot(targetPose.getX() - robot.getPoseEstimate().getX(), targetPose.getY() - robot.getPoseEstimate().getY());

        double absoluteAngleToTarget = Math.atan2(targetPose.getY() - robot.getPoseEstimate().getY(), targetPose.getX() - robot.getPoseEstimate().getY());

        double relativeAngleToTarget = AngleWrap(absoluteAngleToTarget - (robot.getPoseEstimate().getHeading() - Math.toRadians(90)));
        double relativeX;
        double relativeY;

        double v;
        double movementXPower;
        double movementYPower;

        double movementX;
        double movementY;
        double relativeTurnAngle;
        double movementZ;
        
        while(distanceToTarget > allowableDistanceError) {
             distanceToTarget = Math.hypot(targetPose.getX() - robot.getPoseEstimate().getX(), targetPose.getY() - robot.getPoseEstimate().getY());


             absoluteAngleToTarget = Math.atan2(targetPose.getY() - robot.getPoseEstimate().getY(), targetPose.getX() - robot.getPoseEstimate().getY());

             relativeAngleToTarget = AngleWrap(absoluteAngleToTarget - (robot.getPoseEstimate().getHeading() - Math.toRadians(90)));
             relativeX = Math.cos(relativeAngleToTarget) * distanceToTarget;
             relativeY = Math.sin(relativeAngleToTarget) * distanceToTarget;

             v = Math.abs(relativeX) + Math.abs(relativeY);
             movementXPower = relativeX / v;
             movementYPower = relativeY / v;

             movementX = movementXPower * movementSpeed;
             movementY = movementYPower * movementSpeed;

             relativeTurnAngle = relativeAngleToTarget - Math.toRadians(180) + targetPose.getHeading();
             movementZ = Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1) * turnSpeed;

            if(distanceToTarget < nonAllowableTurnDist) {
                movementZ = 0;
            }


            robot.driveMecanum(movementX, movementY, movementZ);
            robot.update();
        }
    }

    public CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius) {
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for(int i = 0; i < pathPoints.size() - 1; i++) {
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());



            double closestAngle = 100000;

            for (Point thisIntersection : intersections) {
                double angle = Math.atan2(thisIntersection.y - robot.getPoseEstimate().getY(), thisIntersection.x - robot.getPoseEstimate().getX());
                double deltaAngle = Math.abs(MathFunctions.AngleWrap(angle - robot.getPoseEstimate().getHeading()));

                if(deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
    }

    public void followCurve (ArrayList<CurvePoint> allPoints, double followAngle) {

        CurvePoint followMe = getFollowPointPath(allPoints, new Point(robot.getPoseEstimate().getX(), robot.getPoseEstimate().getY()) ,
                allPoints.get(0).followDistance);

        goToPosition(new Pose2d(followMe.x,followMe.y, followAngle), followMe.moveSpeed, followMe.turnSpeed, 10, 10); // important thing here
    }


}
