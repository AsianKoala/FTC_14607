package org.firstinspires.ftc.teamcode.control.localization;



import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.SignaturePose;

import java.util.ArrayList;
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
public class WorkingOdometry extends TwoTrackingWheelLocalizer {
    public static final double TICKS_PER_INCH = 1103.8839;

    // 6.75, 4.25, 6.75, -4.25
//    public static double PARALLEL_X = -6.75; // X is the up and down direction
//    public static double PARALLEL_Y = -4.25; // Y is the strafe direction
//
//    public static double PERPENDICULAR_X = -6.75;
//    public static double PERPENDICULAR_Y = 4.25;

    private final static double PARALLEL_X = 6.75;
    private final static double PARALLEL_Y = 4.25;
    private final static double PERPENDICULAR_X = 6.75;
    private final static double PERPENDICULAR_Y = -4.25;

    private Pose currPose;
    private final Pose currVel;
    private final ArrayList<SignaturePose> prevPoses;

    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private final Encoder parallelEncoder, perpendicularEncoder;

    public WorkingOdometry(HardwareMap hardwareMap, Pose startPose) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));


        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftIntake"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightIntake"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        parallelEncoder.setDirection(Encoder.Direction.REVERSE);
        perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);

        currPose = new Pose(startPose);
        setPoseEstimate(new Pose2d(startPose.x, startPose.y, startPose.heading));
        currVel = new Pose();
        prevPoses = new ArrayList<>();
        prevPoses.add(new SignaturePose(startPose, System.currentTimeMillis()));
    }

    public static double encoderTicksToInches(double ticks) {
        return ticks / TICKS_PER_INCH;
    }

    @Override
    public double getHeading() {
        return currPose.heading;
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

    @Override
    public void update() {

    }

    public Pose[] realUpdate(double angle) {
        super.update();
        currPose.heading = angle;
        currPose = new Pose(getPoseEstimate());
        prevPoses.add(new SignaturePose(currPose, System.currentTimeMillis()));
        if (prevPoses.size() > 1) {
            int oldIndex = Math.max(0, prevPoses.size() - 6);
            SignaturePose old = prevPoses.get(oldIndex);
            SignaturePose cur = prevPoses.get(prevPoses.size() - 1);
            double scale = (double) (cur.sign - old.sign) / (1000);
            currVel.set(cur.subtract(old).multiply(new Pose(1 / scale)));
        }
        return new Pose[]{currPose, currVel};
    }



    @SuppressLint("DefaultLocale")
    @Override
    public String toString() {
        return String.format("(%.1f, %.1f, %.1f)", currPose.x, currPose.y, Math.toDegrees(currPose.heading));
    }
}