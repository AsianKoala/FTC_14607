package org.firstinspires.ftc.teamcode.movement;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.opmodes.Robot;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.jetbrains.annotations.NotNull;


public class Odometry {
    public static final double TICKS_PER_INCH = 1103.8839;
public static Robot opMode;
    private int prevVertical;
    private int prevHorizontal;
    private double prevHeading;

    public static double startHeading;
    public static Pose currentPosition;

    private final OdometrySet odometrySet;


    public Odometry(Pose start, OdometrySet odometrySet) {
        startHeading = start.heading;
        prevHorizontal = 0;
        prevVertical = 0;
        prevHeading = startHeading;
        this.odometrySet = odometrySet;

        currentPosition = start;
    }

    public void setStart(Pose start) {
        startHeading = start.heading;
        prevHeading = startHeading;
        currentPosition = start;
    }

    // no real reason to use this tbh
    public void setGlobalPosition(Point newPosition) {
        currentPosition = new Pose(newPosition, currentPosition.heading);
    }

    public void update(double heading) {
//        double deltaY = (odometrySet.getVerticalTicks() - prevVertical) / TICKS_PER_INCH;
//        double deltaX = (odometrySet.getHorizontalTicks() - prevHorizontal) / TICKS_PER_INCH;
//        double deltaAngle = MathUtil.angleWrap(heading - prevHeading);
//
//        double newHeading = MathUtil.angleWrap(currentPosition.heading + deltaAngle);
//        currentPosition.x += - (Math.cos(newHeading) * deltaY) + (Math.sin(newHeading) * deltaX);
//        currentPosition.y += - (Math.sin(newHeading) * deltaY) - (Math.cos(newHeading) * deltaX);
//        currentPosition.heading = newHeading;
//
//        prevHorizontal = odometrySet.getHorizontalTicks();
//        prevVertical = odometrySet.getVerticalTicks();
//        prevHeading = currentPosition.heading;

        
        double currXEncoder = odometrySet.getHorizontalTicks();
        double currYEncoder = odometrySet.getVerticalTicks();

        double deltaAngle = MathUtil.angleWrap(heading - prevHeading);
        currentPosition.heading = MathUtil.angleWrap(currentPosition.heading + deltaAngle);

        double xEncoderDelta = currXEncoder - prevHorizontal;
        double yEncoderDelta = currYEncoder - prevVertical;
        double xWheelDelta = xEncoderDelta / TICKS_PER_INCH;
        double yWheelDelta = yEncoderDelta / TICKS_PER_INCH;

        double xTrackWidth = 8;
        double yTrackWidth = 8;

        double xPrediction = xTrackWidth * deltaAngle;
        double yPrediction = yTrackWidth * deltaAngle;

        double r_x = xWheelDelta - xPrediction;
        double r_y = yWheelDelta - yPrediction;

        updatePosition(new Pose(r_x, r_y, deltaAngle));

        prevHorizontal = odometrySet.getHorizontalTicks();
        prevVertical = odometrySet.getVerticalTicks();
        prevHeading = currentPosition.heading;
    }


    private void updatePosition(Pose deltaPose) {
        double dtheta = deltaPose.heading;
        double sineTerm, cosTerm;
        if (approxEquals(dtheta, 0)) {
            sineTerm = 1.0 - dtheta * dtheta / 6.0;
            cosTerm = dtheta / 2.0;
        } else {
            sineTerm = Math.sin(dtheta) / dtheta;
            cosTerm = (1 - Math.cos(dtheta)) / dtheta;
        }

        Point fieldPositionDelta = new Point(
                sineTerm * deltaPose.x - cosTerm * deltaPose.y,
                cosTerm * deltaPose.x + sineTerm * deltaPose.y
        );

        Pose fieldPoseDelta = new Pose(fieldPositionDelta.rotated(currentPosition.heading), deltaPose.heading);
        currentPosition.add(fieldPoseDelta);
    }
    public static final double EPSILON = 1e-6;
    public static boolean approxEquals(double d1, double d2) {
        if (Double.isInfinite(d1)) {
            // Infinity - infinity is NaN, so we need a special case
            return d1 == d2;
        } else {
            return Math.abs(d1 - d2) < EPSILON;
        }
    }





    @NotNull
    public String toString() {
        return "curr odom readings: " + currentPosition.toString();
    }
}