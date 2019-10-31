package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.acmerobotics.dashboard.*;
import org.firstinspires.ftc.teamcode.ppProject.RobotUtilities.MovementVars;
import org.firstinspires.ftc.teamcode.ppProject.company.Range;

/**
 *  Created on 9/17 by Neil M
 *  Updated 9/17
 */


@Config
public class HouseFly_Hardware {

    // Drivetrain

    public DcMotor frontLeft = null;
    public DcMotor backRight = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;

    public DcMotor succysucky = null; // intake 2
    public DcMotor suckysuccy = null; // intake 1

    public DcMotor verticalLeftEncoder = null;
    public DcMotor verticalRightEncoder = null;
    public DcMotor horizontalEncoder = null;
    
    
    public BNO055IMU imu = null; // we're going to use this for calibration and thats basically it lul

    // numbers here

    public static final double triggerThreshold = 0.25;
    public static final double COUNTS_PER_INCH = 307.699557;



    // Local OpMode members
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public HouseFly_Hardware() {
    }

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        // Define and init stuff

        frontLeft = hwMap.dcMotor.get("fl");
        backLeft = hwMap.dcMotor.get("bl");
        frontRight = hwMap.dcMotor.get("fr");
        backRight = hwMap.dcMotor.get("br");
        succysucky = hwMap.dcMotor.get("succysucky");
        suckysuccy = hwMap.dcMotor.get("suckysuccy");
        verticalLeftEncoder = hwMap.dcMotor.get("verticalLeftEncoder");
        verticalRightEncoder = hwMap.dcMotor.get("verticalRightEncoder");
        horizontalEncoder = hwMap.dcMotor.get("horizontalEncoder");

        imu = hwMap.get(BNO055IMU.class, "revIMU");
        imu.initialize(new BNO055IMU.Parameters());

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        suckysuccy.setPower(0);
        succysucky.setPower(0);


        // set motor settings

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        succysucky.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        suckysuccy.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        suckysuccy.setDirection(DcMotor.Direction.REVERSE);

    }
//
    public void resetEncoders() {

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void resetDriveEncoders() {

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void stopDrive() {

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void stomach(double power) {
        succysucky.setPower(power);
        suckysuccy.setPower(power);
    }

    public void vomit() {
        stomach(-1);
    }

    public void suck() {
        stomach(1);
    }

    public void pauseStomach() {
        stomach(0);
    }



    public void driveLimitless(double left, double right) {
        frontLeft.setPower(left);
        frontRight.setPower(right);
        backLeft.setPower(left);
        backRight.setPower(right);
    }

    public int getHeading(){
        return -(int)Math.floor(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
    }




}
