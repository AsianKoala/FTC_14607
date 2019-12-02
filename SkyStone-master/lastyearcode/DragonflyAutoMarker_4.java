package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.code.Auto.roadrunner.util.DashboardUtil;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "Dragonfly Marker MP Test_2", group = "Dragonfly")
public class DragonflyAutoMarker_4 extends LinearOpMode {

    HardwareDragonflyMP drive = new HardwareDragonflyMP();
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();


        drive.init(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder()
                .turn(Math.toRadians(90))
                .waitFor(1)
                .forward(12)
                .waitFor(1)
                .turn(Math.toRadians(90))
                .waitFor(1)
                .forward(12)
                .waitFor(1)
                .turn(Math.toRadians(90))
                .waitFor(1)
                .forward(12)
                .waitFor(1)
                .turn(Math.toRadians(90))
                .waitFor(1)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);
        while (!isStopRequested() && drive.isFollowingTrajectory()) {
            Pose2d currentPose = drive.getPoseEstimate();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            packet.put("x", currentPose.getX());
            packet.put("y", currentPose.getY());
            packet.put("heading", currentPose.getHeading());

            fieldOverlay.setStrokeWidth(4);
            fieldOverlay.setStroke("green");
            DashboardUtil.drawSampledTrajectory(fieldOverlay, trajectory);

            fieldOverlay.setFill("blue");
            fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

            dashboard.sendTelemetryPacket(packet);

            drive.update();
        }
    }
}