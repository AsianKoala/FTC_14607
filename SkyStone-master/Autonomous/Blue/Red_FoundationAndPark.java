package org.firstinspires.ftc.teamcode.treamcodde.Autonomous.Blue;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.code.Auto.roadrunner.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.treamcodde.HouseFly;

import static java.lang.Math.toRadians;

@Autonomous(name = "RED_Foundation & Park")
public class Red_FoundationAndPark extends LinearOpMode {


    HouseFly robot = new HouseFly(hardwareMap);

    @Override
    public void runOpMode() {
        PIDCoefficients translationPID = new PIDCoefficients(5, 0, 0);
        PIDCoefficients headingPID = new PIDCoefficients(2, 0, 0);
        HolonomicPIDVAFollower follower = new HolonomicPIDVAFollower(translationPID, translationPID, headingPID);


        final MecanumConstraints constraints = DriveConstants.MECANUM_CONSTRAINTS;
        robot.setPoseEstimate(new Pose2d(36,-48, toRadians(90)));

        Trajectory toFoundation = new TrajectoryBuilder(robot.getPoseEstimate(), constraints)
                .splineTo(new Pose2d(50, -24, toRadians(90)))
                .build();

        Trajectory toDepot = new TrajectoryBuilder(robot.getPoseEstimate(), constraints)
                .back(40)
                .build();

        Trajectory park = new TrajectoryBuilder(robot.getPoseEstimate(), constraints)
                .strafeLeft(50)
                .build();

        waitForStart();

        follower.followTrajectory(toFoundation);
        while(robot.isBusy()) {
            DriveSignal signal = follower.update(robot.getPoseEstimate());
        }

        robot.grabFoundation();
        sleep(250);

        follower.followTrajectory(toDepot);
        while(robot.isBusy()) { DriveSignal signal = follower.update(robot.getPoseEstimate()); }

        robot.ungrabFoundation();
        sleep(250);

        follower.followTrajectory(park);
        while(robot.isBusy()) { DriveSignal signal = follower.update(robot.getPoseEstimate()); }

    }
}
