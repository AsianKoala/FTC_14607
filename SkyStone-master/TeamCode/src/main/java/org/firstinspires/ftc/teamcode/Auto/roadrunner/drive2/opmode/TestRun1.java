package org.firstinspires.ftc.teamcode.Auto.roadrunner.drive2.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import kotlin.Unit;
import org.firstinspires.ftc.teamcode.Auto.roadrunner.drive2.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.HelperClasses.FireFly;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Autonomous(group = "drive")
public class TestRun1 extends LinearOpMode {

    private double startX = -32.5;
    private double startY = -62;
    private double startZ = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new FireFly(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d(-32.5, -62.5, 90));
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-24, -48, 90))
                        .splineTo(new Pose2d(-44, -22, 160))
                        .reverse()
//                        .addMarker(2.0){return;}
                        .splineTo(new Pose2d(24, -36, 180))
                        .setReversed(false)
                        .splineTo(new Pose2d(-48, -36, 180))
                        .splineTo(new Pose2d(-62, -24, 160))
                        .reverse()
                        .splineTo(new Pose2d(-24, -36, 180))
                        .splineTo(new Pose2d(48, -36, 180))
//                        .setReversed(false)
//                        .splineTo(new Pose2d(48, -36, 90))
//                        .splineTo(new Pose2d(48, -24, 90))
                        .build()
        );

    }
}
