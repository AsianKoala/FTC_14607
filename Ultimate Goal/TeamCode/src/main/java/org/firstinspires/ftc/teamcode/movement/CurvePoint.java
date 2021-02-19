package org.firstinspires.ftc.teamcode.movement;

import org.firstinspires.ftc.teamcode.control.Functions;
import org.firstinspires.ftc.teamcode.util.Point;

public class CurvePoint {
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
        function = thisPoint.function;
    }

    public Point toPoint(){
        return new Point(x,y);
    }
    public void setPoint(Point p){
        x = p.x;
        y = p.y;
    }
}
