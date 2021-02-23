package org.firstinspires.ftc.teamcode.movement;

import org.firstinspires.ftc.teamcode.control.Functions;
import org.firstinspires.ftc.teamcode.util.Point;

public class CurvePoints {
    public static class BaseCurvePoint {
        public double x;
        public double y;
        public double followDistance;
        public double pointLength;
        public Functions.function function;

        public BaseCurvePoint(double x, double y,
                              double followDistance, double pointLength, Functions.function function) {
            this.x = x;
            this.y = y;
            this.followDistance = followDistance;
            this.pointLength = pointLength;
            this.function = function;
        }

        public BaseCurvePoint(BaseCurvePoint thisPoint) {
            x = thisPoint.x;
            y = thisPoint.y;
            followDistance = thisPoint.followDistance;
            pointLength = thisPoint.pointLength;
            function = thisPoint.function;
        }

        public Point toPoint() {
            return new Point(x, y);
        }

        public void setPoint(Point p) {
            x = p.x;
            y = p.y;
        }
    }


    public static class HeadingControlledCurvePoint extends BaseCurvePoint {
        public double targetAngle;

        public HeadingControlledCurvePoint(double x, double y, double followDistance, double pointLength, Functions.function function) {
            super(x, y, followDistance, pointLength, function);
        }

        public HeadingControlledCurvePoint(BaseCurvePoint thisPoint) {
            super(thisPoint);
        }
    }
}
