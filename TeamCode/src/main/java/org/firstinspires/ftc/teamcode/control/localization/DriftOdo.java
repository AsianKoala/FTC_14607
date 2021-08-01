package org.firstinspires.ftc.teamcode.control.localization;

import org.firstinspires.ftc.teamcode.control.system.Azusa;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.openftc.revextensions2.RevBulkData;

public class DriftOdo {
    // turn 90
    //
    //
    final Point tracker_coeffs = new Point(8672 / TICKS_PER_INCH, -7158 / TICKS_PER_INCH);

    private static final double TICKS_PER_INCH = 1103.8839;

    private int prevVertical;
    private int prevHorizontal;
    private double prevHeading;

    public static double startHeading;

    private final Pose currentPosition;

    public DriftOdo(Pose start) {
        startHeading = start.h;
        prevHorizontal = 0;
        prevVertical = 0;
        prevHeading = startHeading;

        currentPosition = start;
    }

    public Pose update(Azusa azusa, double heading, RevBulkData data) {
        int horiz_port = 0;
        int curr_horiz = data.getMotorCurrentPosition(horiz_port);
        int parallel_port = 2;
        int curr_vert = data.getMotorCurrentPosition(parallel_port);
        double deltaX = (curr_horiz - prevHorizontal) / TICKS_PER_INCH;
        double deltaY = (curr_vert - prevVertical) / TICKS_PER_INCH;
        double deltaAngle = MathUtil.angleWrap(heading - prevHeading);

        Point tracker_scaled = tracker_coeffs.scale(1 / (Math.PI / 2));
        double corrected_delta_x = deltaX - deltaAngle * tracker_scaled.x;
        double corrected_delta_y = deltaY - deltaAngle * tracker_scaled.y;

        currentPosition.x += -(Math.cos(heading) * corrected_delta_y) + (Math.sin(heading) * corrected_delta_x);
        currentPosition.y += -(Math.sin(heading) * corrected_delta_y) - (Math.cos(heading) * corrected_delta_x);
        currentPosition.h = heading + startHeading;

        azusa.telemetry.addData("curr x", curr_horiz);
        azusa.telemetry.addData("curr y", curr_vert);
        azusa.telemetry.addData("delta x", corrected_delta_x);
        azusa.telemetry.addData("delta y", corrected_delta_y);
        azusa.telemetry.addData("delta H", deltaAngle);
        azusa.telemetry.addData("last horizontal", prevHorizontal);
        azusa.telemetry.addData("last vertical", prevVertical);
        prevHorizontal = curr_horiz;
        prevVertical = curr_vert;
        prevHeading = currentPosition.h;
        return currentPosition;
    }
}
