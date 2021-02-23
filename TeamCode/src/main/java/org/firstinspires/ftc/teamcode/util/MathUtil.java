package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

public class
MathUtil {
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


    public static Point clipToLine(double lineX1, double lineY1, double lineX2, double lineY2,
                                   double robotX, double robotY) {
        if (lineX1 == lineX2) {
            lineX1 = lineX2 + 0.01;//nah
        }
        if (lineY1 == lineY2) {
            lineY1 = lineY2 + 0.01;//nah
        }

        //calculate the slope of the line
        double m1 = (lineY2 - lineY1) / (lineX2 - lineX1);
        //calculate the slope perpendicular to this line
        double m2 = (lineX1 - lineX2) / (lineY2 - lineY1);

        //clip the robot's position to be on the line
        double xClipedToLine = ((-m2 * robotX) + robotY + (m1 * lineX1) - lineY1) / (m1 - m2);
        double yClipedToLine = (m1 * (xClipedToLine - lineX1)) + lineY1;
        return new Point(xClipedToLine, yClipedToLine);
    }

    public static ArrayList<Point> lineCircleIntersection(double circleX, double circleY, double r,
                                                          double lineX1, double lineY1,
                                                          double lineX2, double lineY2) {
        //make sure the points don't exactly line up so the slopes work
        if (Math.abs(lineY1 - lineY2) < 0.003) {
            lineY1 = lineY2 + 0.003;
        }
        if (Math.abs(lineX1 - lineX2) < 0.003) {
            lineX1 = lineX2 + 0.003;
        }

        //calculate the slope of the line
        double m1 = (lineY2 - lineY1) / (lineX2 - lineX1);

        //the first coefficient in the quadratic
        double quadraticA = 1.0 + pow(m1, 2);

        //shift one of the line's points so it is relative to the circle
        double x1 = lineX1 - circleX;
        double y1 = lineY1 - circleY;


        //the second coefficient in the quadratic
        double quadraticB = (2.0 * m1 * y1) - (2.0 * pow(m1, 2) * x1);

        //the third coefficient in the quadratic
        double quadraticC = ((pow(m1, 2) * pow(x1, 2)) - (2.0 * y1 * m1 * x1) + pow(y1, 2) - pow(r, 2));


        ArrayList<Point> allPoints = new ArrayList<>();


        //this may give an error so we use a try catch
        try {
            //now solve the quadratic equation given the coefficients
            double xRoot1 = (-quadraticB + sqrt(pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);

            //we know the line equation so plug into that to get root
            double yRoot1 = m1 * (xRoot1 - x1) + y1;


            //now we can add back in translations
            xRoot1 += circleX;
            yRoot1 += circleY;

            //make sure it was within range of the segment
            double minX = Math.min(lineX1, lineX2);
            double maxX = Math.max(lineX1, lineX2);
            if (xRoot1 > minX && xRoot1 < maxX) {
                allPoints.add(new Point(xRoot1, yRoot1));
            }

            //do the same for the other root
            double xRoot2 = (-quadraticB - sqrt(pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);

            double yRoot2 = m1 * (xRoot2 - x1) + y1;
            //now we can add back in translations
            xRoot2 += circleX;
            yRoot2 += circleY;

            //make sure it was within range of the segment
            if (xRoot2 > minX && xRoot2 < maxX) {
                allPoints.add(new Point(xRoot2, yRoot2));
            }
        } catch (Exception e) {
            //if there are no roots
        }
        return allPoints;
    }
}
