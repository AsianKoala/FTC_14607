package org.firstinspires.ftc.teamcode.control.localization;

import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.jetbrains.annotations.NotNull;


public class Odometry {
    // 8192 ticks per revolution
    // wheels are 60mm, or 2.3622 inches diameter
    // 2.3622 * pi = 7.42107016631 circumference
    // 8192 / 7.42107016631 = ticks per inch
    public static final double TICKS_PER_INCH = 1103.8839;

    private int prevVertical;
    private int prevHorizontal;
    private double prevHeading;

    public static double startHeading;
    public static Pose currentPosition;

    private final OdometrySet odometrySet;


    public Odometry(Pose start, OdometrySet odometrySet) {
        startHeading = start.h;
        prevHorizontal = 0;
        prevVertical = 0;
        prevHeading = startHeading;
        this.odometrySet = odometrySet;

        currentPosition = start;
    }

    public void setStart(Pose start) {
        startHeading = start.h;
        prevHeading = startHeading;
        currentPosition = start;
    }

    // very very dangerous
    public void setGlobalPosition(Point newPosition) {
        currentPosition = new Pose(newPosition, currentPosition.h);
    }

    public void update(double heading) {
        double deltaY = (odometrySet.getVerticalTicks() - prevVertical) / TICKS_PER_INCH;
        double deltaX = (odometrySet.getHorizontalTicks() - prevHorizontal) / TICKS_PER_INCH;
        double deltaAngle = MathUtil.angleWrap(heading - prevHeading);

//        double newHeading = MathUtil.angleWrap(currentPosition.h + deltaAngle);
        currentPosition.x += -(Math.cos(heading) * deltaY) + (Math.sin(heading) * deltaX);
        currentPosition.y += -(Math.sin(heading) * deltaY) - (Math.cos(heading) * deltaX);
        currentPosition.h = heading + startHeading;

        prevHorizontal = odometrySet.getHorizontalTicks();
        prevVertical = odometrySet.getVerticalTicks();
        prevHeading = currentPosition.h;
    }


    @NotNull
    public String toString() {
        return "v: " + odometrySet.getVerticalTicks() + " , " + "h: " + odometrySet.getHorizontalTicks();
    }
}