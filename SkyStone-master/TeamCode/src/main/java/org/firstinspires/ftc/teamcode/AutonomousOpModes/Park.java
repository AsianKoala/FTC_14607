package org.firstinspires.ftc.teamcode.AutonomousOpModes;

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

@Autonomous(name = "ParkOnly", group = "")
public class Park extends LinearOpMode {


    HouseFly robot = new HouseFly(hardwareMap);


    @Override
    public void runOpMode() {
        robot.setMotorPowers(0.5,0.5,0.5,0.5);
        sleep(100);
    }
}
