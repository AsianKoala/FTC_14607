package org.firstinspires.ftc.teamcode.movement;

import org.firstinspires.ftc.teamcode.control.Functions;
import org.firstinspires.ftc.teamcode.util.Point;

public class CurvePoint implements Cloneable {
    public double x;
    public double y;
    public double moveSpeed;
    public double turnSpeed;
    public double followDistance;
    public double slowDownTurnRadians;
    public double slowDownTurnAmount;
    public double pointLength;
    public Functions.function function;

    public CurvePoint(double x, double y, double moveSpeed, double turnSpeed,
                      double followDistance,double pointLength, double slowDownTurnRadians, double slowDownTurnAmount, Functions.function function){
        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.pointLength = pointLength;
        this.slowDownTurnRadians = slowDownTurnRadians;
        this.slowDownTurnAmount = slowDownTurnAmount;
        this.function = function;
    }

    public CurvePoint(CurvePoint thisPoint) {
        x = thisPoint.x;
        y = thisPoint.y;
        moveSpeed = thisPoint.moveSpeed;
        turnSpeed = thisPoint.turnSpeed;
        followDistance = thisPoint.followDistance;
        slowDownTurnRadians = thisPoint.slowDownTurnRadians;
        slowDownTurnAmount = thisPoint.slowDownTurnAmount;
        pointLength = thisPoint.pointLength;

    }

    public Point toPoint(){
        return new Point(x,y);
    }
    public void setPoint(Point p){
        x = p.x;
        y = p.y;
    }

    @Override
    public CurvePoint clone() {
        try {
            return (CurvePoint) super.clone();
        } catch (Exception ignored) {}
        return null;
    }


    public static class hardwareCurvePoint extends CurvePoint {
        public hardwareCurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance, double pointLength, double slowDownTurnRadians, double slowDownTurnAmount, Functions.hardwareFunction function) {
            super(x, y, moveSpeed, turnSpeed, followDistance, pointLength, slowDownTurnRadians, slowDownTurnAmount, function);
        }
    }

    public static class headingControlledCurvePoint extends CurvePoint {
        public headingControlledCurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance, double pointLength, double slowDownTurnRadians, double slowDownTurnAmount, Functions.headingControlledFunction function) {
            super(x, y, moveSpeed, turnSpeed, followDistance, pointLength, slowDownTurnRadians, slowDownTurnAmount, function);
        }
    }

    public static class pointToPointCurvePoint extends CurvePoint {
        public pointToPointCurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance, double pointLength, double slowDownTurnRadians, double slowDownTurnAmount, Functions.pointToPointFunction function) {
            super(x, y, moveSpeed, turnSpeed, followDistance, pointLength, slowDownTurnRadians, slowDownTurnAmount, function);
        }
    }

    public static class basicCurvePoint extends CurvePoint {
        public basicCurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance, double pointLength, double slowDownTurnRadians, double slowDownTurnAmount, Functions.function function) {
            super(x, y, moveSpeed, turnSpeed, followDistance, pointLength, slowDownTurnRadians, slowDownTurnAmount, function);
        }
    }
//
//    public static class complexCurvePoint extends CurvePoint {
//        public complexCurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance, double pointLength, double slowDownTurnRadians, double slowDownTurnAmount, Functions.complexFunction function) {
//            super(x, y, moveSpeed, turnSpeed, followDistance, pointLength, slowDownTurnRadians, slowDownTurnAmount, function);
//        }
//    }
}
