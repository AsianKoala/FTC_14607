package org.firstinspires.ftc.teamcode.movement;

import org.firstinspires.ftc.teamcode.control.Modifier;
import org.firstinspires.ftc.teamcode.util.Point;

public class CurvePoint {
    public double x;
    public double y;
    public double followDistance;
    public double pointLength;
    public Modifier.modifier modifier;

    public CurvePoint(double x, double y,
                          double followDistance, double pointLength, Modifier.modifier modifier) {
        this.x = x;
        this.y = y;
        this.followDistance = followDistance;
        this.pointLength = pointLength;
        this.modifier = modifier;
    }

    public CurvePoint(CurvePoint thisPoint) {
        x = thisPoint.x;
        y = thisPoint.y;
        followDistance = thisPoint.followDistance;
        pointLength = thisPoint.pointLength;
        modifier = thisPoint.modifier;
    }

    public Point toPoint() {
        return new Point(x, y);
    }

    public void setPoint(Point p) {
        x = p.x;
        y = p.y;
    }
}
