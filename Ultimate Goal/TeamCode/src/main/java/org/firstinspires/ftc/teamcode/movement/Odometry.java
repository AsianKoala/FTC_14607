package org.firstinspires.ftc.teamcode.movement;

import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.jetbrains.annotations.NotNull;


public class Odometry {
    public static final double TICKS_PER_INCH = 1103.8839;

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

        double deltaLeftVertical = (odometrySet.getVerticalTicks() - prevVertical) / TICKS_PER_INCH;
        double deltaAngle = MathUtil.angleWrap(heading - prevHeading);

        double deltaVirtualRightVertical = (deltaAngle * 11.5 + deltaLeftVertical) / TICKS_PER_INCH;
        double relativeY = (deltaLeftVertical + deltaVirtualRightVertical) / 2.0;

        double relativeX = (odometrySet.getHorizontalTicks() - prevHorizontal) / TICKS_PER_INCH;
        relativeX = (relativeX + (deltaAngle * 12.3 + relativeX)) / 2.0;

        currentPosition.heading = MathUtil.angleWrap(currentPosition.heading + deltaAngle);
        currentPosition.x += - (Math.cos(currentPosition.heading) * relativeY) + (Math.sin(currentPosition.heading) * relativeX);
        currentPosition.y += - (Math.sin(currentPosition.heading) * relativeY) - (Math.sin(currentPosition.heading) * relativeX);

        double xAngleScale = -2.5 / Math.PI;
        double yAngleScale = -4.4 / Math.PI;
        currentPosition.x += MathUtil.angleWrap(currentPosition.heading - startHeading) * xAngleScale;
        currentPosition.y += MathUtil.angleWrap(currentPosition.heading - startHeading) * yAngleScale;


        prevHorizontal = odometrySet.getHorizontalTicks();
        prevVertical = odometrySet.getVerticalTicks();
        prevHeading = currentPosition.heading;
    }


    @NotNull
    public String toString() {
        return "curr odom readings: " + currentPosition.toString();
    }
}