package org.firstinspires.ftc.teamcode.control.localization;

import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.SignaturePose;
import org.jetbrains.annotations.NotNull;

import java.util.LinkedList;


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
    private final Pose currentPosition, currentVel;
    private final LinkedList<SignaturePose> prevPoses;

    public Odometry(Pose start) {
        startHeading = start.h;
        prevPerp = 0;
        prevParallel = 0;
        prevHeading = startHeading;

        currentPosition = start;
        currentVel = new Pose();
        prevPoses = new LinkedList<>();
        prevPoses.add(new SignaturePose(currentPosition));
    }

    public Pose[] update(int parallel, int perp, double heading) {
        double deltaY = (parallel - prevParallel) / TICKS_PER_INCH;
        double deltaX = (perp - prevPerp) / TICKS_PER_INCH;
        double deltaAngle = MathUtil.angleWrap(heading - prevHeading);

        double newHeading = MathUtil.angleWrap(currentPosition.h + deltaAngle);
        currentPosition.x += - (Math.cos(newHeading) * deltaY) + (Math.sin(newHeading) * deltaX);
        currentPosition.y += - (Math.sin(newHeading) * deltaY) - (Math.cos(newHeading) * deltaX);
        currentPosition.h = newHeading;

        if(prevPoses.size() > 1) {
            int oldIndex = Math.max(0, prevPoses.size() - 6);
            SignaturePose old = prevPoses.get(oldIndex);
            SignaturePose cur = prevPoses.get(prevPoses.size() - 1);
            double scale = (double) (cur.sign - old.sign) / (1000);
            currentVel.set(cur.minus(old).multiply(new Pose(1 / scale)));
        }

        prevPerp = parallel;
        prevParallel = perp;
        prevHeading = currentPosition.h;
        prevPoses.add(new SignaturePose(currentPosition));
        return new Pose[]{currentPosition, currentVel};
    }


    @NotNull
    public String toString() {
        return "v: " + prevParallel + " , " + "h: " + prevPerp;
    }
}