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

    private int prevParallel;
    private int prevPerp;
    private double prevHeading;

    public double startHeading;
    private Pose currentPosition;

    public Odometry(Pose start) {
        startHeading = start.h;
        prevPerp = 0;
        prevParallel = 0;
        prevHeading = startHeading;
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

    public Pose update(int parallel, int perp, double heading) {
        double deltaY = (parallel - prevParallel) / TICKS_PER_INCH;
        double deltaX = (perp - prevPerp) / TICKS_PER_INCH;
        double deltaAngle = MathUtil.angleWrap(heading - prevHeading);

        double newHeading = MathUtil.angleWrap(currentPosition.h + deltaAngle);
        currentPosition.x += - (Math.cos(newHeading) * deltaY) + (Math.sin(newHeading) * deltaX);
        currentPosition.y += - (Math.sin(newHeading) * deltaY) - (Math.cos(newHeading) * deltaX);
        currentPosition.h = newHeading;

        prevPerp = parallel;
        prevParallel = perp;
        prevHeading = currentPosition.h;
        return currentPosition;
    }


    @NotNull
    public String toString() {
        return "v: " + prevParallel + " , " + "h: " + prevPerp;
    }
}