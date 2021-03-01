package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;

public class MathUtil {
    public static final double EPSILON = 1e-6;

    public static double angleWrap(double angle) {
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    public static boolean epsilonEquals(double d1, double d2) {
        if (Double.isInfinite(d1)) {
            // Infinity - infinity is NaN, so we need a special case
            return d1 == d2;
        } else {
            return Math.abs(d1 - d2) < EPSILON;
        }
    }

    public static int sgn(double x) {
        return x > 0 ? 1 : -1;
    }

    /**
     * Returns the closest intersection point to the end of a line segment created through the intersection of a line and circle.
     * The main purpose of this is for pure pursuit but I'll prob implement it into our goToPosition algorithm.
     * For pure pursuit use, c would be the clipped robot point, startPoint would be the current segment start point,
     * endPoint would be the current segment end point, and radius would be our follow distance
     *
     * @param c          center point of circle
     * @param startPoint start point of the line segment
     * @param endPoint   end point of the line segment
     * @param radius     radius of the circle
     * @return intersection point closest to endPoint
     * @see <a href="https://mathworld.wolfram.com/Circle-LineIntersection.html">https://mathworld.wolfram.com/Circle-LineIntersection.html</a>
     */
    public static Point circleLineIntersection(Point c, Point startPoint, Point endPoint, double radius) {
        Point start = new Point(startPoint.x - c.x, startPoint.y - c.y);
        Point end = new Point(endPoint.x - c.x, endPoint.y - c.y);

        double dx = end.x - start.x;
        double dy = end.y - start.y;
        double dr = Math.hypot(dx, dy);
        double D = start.x * end.y - end.x * start.y;
        double discriminant = radius * radius * dr * dr - D * D;

        // discriminant = 0 for 1 intersection, >0 for 2
        ArrayList<Point> intersections = new ArrayList<>();
        double xLeft = D * dy;
        double yLeft = -D * dx;
        double xRight = sgn(dy) * dx * Math.sqrt(discriminant);
        double yRight = Math.abs(dy) * Math.sqrt(discriminant);
        double div = dr * dr;

        if (discriminant == 0) {
            intersections.add(new Point(xLeft / div, yLeft / div));
        } else {
            // add 2 points, one with positive right side and one with negative right side
            intersections.add(new Point(
                    (xLeft + xRight) / div,
                    (yLeft + yRight) / div
            ));

            intersections.add(new Point(
                    (xLeft - xRight) / div,
                    (yLeft - yRight) / div
            ));
        }

        Point closest = new Point(-10000, -10000);
        for (Point p : intersections) { // add circle center offsets
            p.x += c.x;
            p.y += c.y;
            if (p.distance(endPoint) < closest.distance(endPoint))
                closest = p;
        }
        return closest;
    }
}
