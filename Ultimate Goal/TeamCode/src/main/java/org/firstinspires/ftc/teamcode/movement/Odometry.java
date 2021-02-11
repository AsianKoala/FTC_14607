package org.firstinspires.ftc.teamcode.movement;



import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */

/*
center of rotation is (8.75, 6.25)
perp wheel is (4.5, 13)
parallel wheel is (13, 13)
deltas are as follows:
perp wheel: (-4.25, 6.75)
parallel wheel: (4.25, 6.75)
reverse the coords and we get this
perp: (6.75, -4.25)
parallel (6.75, 4.25)

 */
public class Odometry extends TwoTrackingWheelLocalizer {
    public static final double TICKS_PER_INCH = 1103.8839;

    // 6.75, 4.25, 6.75, -4.25
//    public static double PARALLEL_X = -6.75; // X is the up and down direction
//    public static double PARALLEL_Y = -4.25; // Y is the strafe direction
//
//    public static double PERPENDICULAR_X = -6.75;
//    public static double PERPENDICULAR_Y = 4.25;

    public static double PARALLEL_X = 6.75;
    public static double PARALLEL_Y = 4.25;
    public static double PERPENDICULAR_X = 6.75;
    public static double PERPENDICULAR_Y = -4.25;


    public static Pose currentPosition;
    public static Pose startPosition;

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;


    public Odometry(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));


        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightIntake"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftIntake"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return ticks / TICKS_PER_INCH;
    }

    @Override
    public double getHeading() {
        return currentPosition.heading;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()),
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCorrectedVelocity()),
                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity())
        );
    }


    public void updateAngle(double angle) {
        currentPosition.heading = angle;
    }

    public void setStart(Pose start) {
        currentPosition = start;
        startPosition = currentPosition;
    }

    public void setGlobalPosition(Point newPos) {
        currentPosition.x = newPos.x;
        currentPosition.y = newPos.y;
    }


    @Override
    public void update() {
        super.update();
        currentPosition.x = getPoseEstimate().getX() + startPosition.x;
        currentPosition.y = getPoseEstimate().getY() + startPosition.y;
    }


    @NotNull
    @SuppressLint("DefaultLocale")
    @Override
    public String toString() {
        return String.format("(%.1f, %.1f, %.1f)", currentPosition.x, currentPosition.y, Math.toDegrees(currentPosition.heading));
    }
}