package org.firstinspires.ftc.teamcode.AutonomousOpModes.Blue;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.drive.mecanum.HouseFly;

import static java.lang.Math.toRadians;

@Autonomous(name = "BLUE___ Left side to mid park")
public class Blue_LeftSideToMid extends LinearOpMode {


    HouseFly robot = new HouseFly(hardwareMap);


    @Override
    public void runOpMode() {


        PIDCoefficients translationPID = new PIDCoefficients(5, 0, 0);
        PIDCoefficients headingPID = new PIDCoefficients(2, 0, 0);
        HolonomicPIDVAFollower follower = new HolonomicPIDVAFollower(translationPID, translationPID, headingPID);

        final MecanumConstraints constraints = DriveConstants.MECANUM_CONSTRAINTS;
        robot.setPoseEstimate(new Pose2d(12, 60, toRadians(90)));

        Trajectory toMiddle = new TrajectoryBuilder(robot.getPoseEstimate(), constraints)
                .strafeRight(12)
                .build();

        waitForStart();

        robot.followTrajectory(toMiddle);
        while (follower.isFollowing()) {
            DriveSignal signal = follower.update(robot.getPoseEstimate());
        }
    }
}
