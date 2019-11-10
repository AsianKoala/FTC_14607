package org.firstinspires.ftc.teamcode.roadrunner.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.drive.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.drive.Hardware.Intake;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.ArrayList;

@Config
public class Housefly extends MecanumDrive {

    /**
     * init hardware
     */


    // init motors
    public ExpansionHubMotor frontLeft;
    public ExpansionHubMotor frontRight;
    public ExpansionHubMotor backLeft;
    public ExpansionHubMotor backRight;
    public ExpansionHubMotor intakeLeft;
    public ExpansionHubMotor intakeRight;


    // init imu
    public BNO055IMU imu;



    // init subsystems
    public DriveTrain driveTrain;
    public Intake  intake;


}
