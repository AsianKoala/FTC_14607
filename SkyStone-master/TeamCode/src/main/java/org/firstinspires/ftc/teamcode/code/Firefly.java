package org.firstinspires.ftc.teamcode.code;


import android.support.annotation.NonNull;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.util.LynxOptimizedI2cFactory;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.roadrunner.teamcode.drive.DriveConstants.encoderTicksToInches;

public class Firefly extends SampleMecanumDriveBase {
    public ExpansionHubEx master, slave;
    public ExpansionHubMotor frontLeft, frontRight, backLeft, backRight, leftIntake, rightIntake, leftSlide, rightSlide;
    public ExpansionHubServo leftSlam, rightSlam, rotater, gripper, flipper;
    private BNO055IMU imu;

    private ArrayList<ExpansionHubMotor> leftMotors = new ArrayList<>();
    private ArrayList<ExpansionHubMotor> rightMotors = new ArrayList<>();
    private ArrayList<ExpansionHubMotor> driveMotors = new ArrayList<>();





    public final double flipperHome =  0.95;
    public final double flipperOut = 0.25;
    public final double flipperBetween = (flipperHome + flipperOut)/2;
    public final double flipperBetweenBetween = (flipperBetween + flipperOut)/2;
    public final double rotaterHome = 0.279;
    public final double rotaterOut = 0.95;
    public final double gripperHome = 0.41;
    public final double gripperGrip = 0.2;





    public Firefly(HardwareMap hardwareMap) {
        super();


        // connect stuff to phone stuff
        master = hardwareMap.get(ExpansionHubEx.class, "master");
        slave = hardwareMap.get(ExpansionHubEx.class, "follower");
        frontLeft = hardwareMap.get(ExpansionHubMotor.class, "FL");
        backLeft = hardwareMap.get(ExpansionHubMotor.class, "BL");
        backRight = hardwareMap.get(ExpansionHubMotor.class, "BR");
        frontRight = hardwareMap.get(ExpansionHubMotor.class, "FR");
        leftIntake = hardwareMap.get(ExpansionHubMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(ExpansionHubMotor.class, "rightIntake");
        leftSlide = hardwareMap.get(ExpansionHubMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(ExpansionHubMotor.class, "rightSlide");
        leftSlam = hardwareMap.get(ExpansionHubServo.class, "leftSlam");
        rightSlam = hardwareMap.get(ExpansionHubServo.class, "rightSlam");
        rotater = hardwareMap.get(ExpansionHubServo.class, "rotater");
        gripper = hardwareMap.get(ExpansionHubServo.class, "gripper");
        flipper = hardwareMap.get(ExpansionHubServo.class, "flipper");
        imu = hardwareMap.get(BNO055IMU.class, "imu");


        /**
         * add motors to arrays
         */

        driveMotors.add(frontLeft);
        driveMotors.add(frontRight);
        driveMotors.add(backLeft);
        driveMotors.add(backRight);

        leftMotors.add(frontLeft);
        leftMotors.add(backLeft);
        leftMotors.add(leftIntake);
        leftMotors.add(leftSlide);


        /**
         * init imu stuff
         */
        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(master.getStandardModule(), 0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);


        /**
         * init motors
         */


        for(ExpansionHubMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        for(ExpansionHubMotor motor : leftMotors) {
            motor.setDirection(DcMotor.Direction.REVERSE);
        }

        // init slides
        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);
        leftSlide.setMode(ExpansionHubMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(ExpansionHubMotor.RunMode.RUN_TO_POSITION);

        leftIntake.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(ExpansionHubMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = frontLeft.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (ExpansionHubMotor motor : driveMotors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, 1
            ));
        }
    }
    
    
    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData masterBulk = master.getBulkInputData();
        RevBulkData slaveBulk = slave.getBulkInputData();

        if (masterBulk == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelPositions = new ArrayList<>();


        wheelPositions.add(encoderTicksToInches(masterBulk.getMotorCurrentPosition(0)));
        wheelPositions.add(encoderTicksToInches(masterBulk.getMotorCurrentPosition(1)));
        wheelPositions.add(encoderTicksToInches(slaveBulk.getMotorCurrentPosition(0)));
        wheelPositions.add(encoderTicksToInches(slaveBulk.getMotorCurrentPosition(1)));
        return wheelPositions;
    }




    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        frontLeft.setPower(v);
        backLeft.setPower(v1);
        backRight.setPower(v2);
        frontRight.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }


    /**
     * add other stuff here
     */


}
