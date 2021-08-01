package org.firstinspires.ftc.teamcode.control.localization;

import org.firstinspires.ftc.teamcode.control.system.Azusa;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.openftc.revextensions2.RevBulkData;

public class DriftOdo {
    public static final double TICKS_PER_INCH = 1103.8839;
    public static final Point tracker_coeffs = new Point(8272.5 / TICKS_PER_INCH, -8651 / TICKS_PER_INCH); //8672 -7158

    public static final int horiz_port = 0;
    public static final int parallel_port = 2;

    public static double startHeading;
    private final Pose currentPosition;

    public Point curr_wheels;
    public Point delta_scaled;
    public Point tracker_scaled;
    public Point corrected_deltas;

    public Point prev_wheels;
    public double prevHeading;

    public DriftOdo(Pose start) {
        startHeading = start.h;
        prev_wheels = new Point();
        prevHeading = startHeading;

        currentPosition = start;
    }

    public Pose update(Azusa azusa, double heading, RevBulkData data) {
        curr_wheels = new Point(
                data.getMotorCurrentPosition(horiz_port),
                data.getMotorCurrentPosition(parallel_port));
        delta_scaled = (curr_wheels.minus(prev_wheels)).scale(1 / TICKS_PER_INCH);
        double deltaAngle = MathUtil.angleWrap(heading - prevHeading);

        tracker_scaled = tracker_coeffs.scale(1 / (Math.PI / 2));
        corrected_deltas = delta_scaled.minus(tracker_scaled.scale(deltaAngle));

        currentPosition.x += -(Math.cos(heading) * corrected_deltas.y) + (Math.sin(heading) * corrected_deltas.x);
        currentPosition.y += -(Math.sin(heading) * corrected_deltas.y) - (Math.cos(heading) * corrected_deltas.x);
        currentPosition.h = MathUtil.angleWrap(heading + startHeading);

        azusa.telemetry.addData("curr", curr_wheels);
        azusa.telemetry.addData("corrected", corrected_deltas);
        azusa.telemetry.addData("delta H", deltaAngle);
        azusa.telemetry.addData("prev wheels", prev_wheels);

        prev_wheels = curr_wheels;
        prevHeading = currentPosition.h;
        return currentPosition;
    }
}
