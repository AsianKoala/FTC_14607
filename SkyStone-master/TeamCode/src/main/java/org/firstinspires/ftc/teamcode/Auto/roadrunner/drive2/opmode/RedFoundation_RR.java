package org.firstinspires.ftc.teamcode.Auto.roadrunner.drive2.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.HelperClasses.FireFly;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class RedFoundation_RR extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        FireFly drive = new FireFly(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d(38.5, -62.5, 90.0));

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(45.0, -62.5))
                        .lineTo(new Vector2d(45.0, -29.0))
                        // add marker to grab foundation
                        .build()
        );

        drive.grabFoundation();

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .lineTo(new Vector2d(45.0, -38.0))
                        .splineTo(new Pose2d(40.0, -38.0, 0.0))
                        .reverse()
                        .lineTo(new Vector2d(50.0, -38.0))
                        // add marker to push forward for time
                        // add marker to ungrab foundation
                        .build()
        );

        double startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - startTime < 1000 && opModeIsActive()) {
            drive.leftFront.setPower(0.3);
            drive.leftRear.setPower(0.3);
            drive.rightFront.setPower(0.3);
            drive.rightRear.setPower(0.3);
        }
        drive.leftFront.setPower(0);
        drive.leftRear.setPower(0);
        drive.rightFront.setPower(0);
        drive.rightRear.setPower(0);

        drive.ungrabFoundation();

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .strafeTo(new Vector2d(36.0, -60.0))
                        .lineTo(new Vector2d(0.0, -60.0))
                        //strafe for time right against wall
                        .build()
        );

        startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - startTime < 1500 && opModeIsActive()) {
            drive.leftFront.setPower(0.25);
            drive.leftRear.setPower(-0.25);
            drive.rightFront.setPower(0.25);
            drive.rightRear.setPower(-0.25);
        }
        drive.leftFront.setPower(0);
        drive.leftRear.setPower(0);
        drive.rightFront.setPower(0);
        drive.rightRear.setPower(0);

    }
}
