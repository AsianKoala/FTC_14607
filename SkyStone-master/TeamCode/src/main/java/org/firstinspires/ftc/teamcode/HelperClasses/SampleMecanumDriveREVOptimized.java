package org.firstinspires.ftc.teamcode.HelperClasses;

import static org.firstinspires.ftc.teamcode.Auto.roadrunner.drive.DriveConstants.*;


import android.support.annotation.NonNull;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.Auto.roadrunner.drive.mecanum.SampleMecanumDriveBase;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

/*
 * Optimized mecanum drive implementation for REV ExHs. The time savings may significantly improve
 * trajectory following performance with moderate additional complexity.
 */

/**
 * we are only using this class for its methods, DO NOT INIT ANYTHING IN HERE
 */
public class SampleMecanumDriveREVOptimized extends SampleMecanumDriveBase {
    private ExpansionHubEx master, slave;
    public ExpansionHubMotor frontLeft, frontRight, backLeft, backRight;
    private List<ExpansionHubMotor> motors;
    private BNO055IMU imu;

    public SampleMecanumDriveREVOptimized(ArrayList<ExpansionHubMotor> allMotors, BNO055IMU imu, ExpansionHubEx master, ExpansionHubEx slave) {
        super();

        this.imu = imu;
        this.master = master;
        this.slave = slave;


        frontLeft = allMotors.get(0);
        frontRight = allMotors.get(1);
        backLeft = allMotors.get(2);
        backRight = allMotors.get(3);


        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = frontLeft.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (ExpansionHubMotor motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, getMotorVelocityF()
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        // master = right
        RevBulkData bulkData = master.getBulkInputData();
        RevBulkData slaveData = slave.getBulkInputData();

        if (bulkData == null || slaveData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }
        // fl bl fr br
        List<Double> wheelPositions = new ArrayList<>();
        wheelPositions.add(encoderTicksToInches(slaveData.getMotorCurrentPosition(0)));
        wheelPositions.add(encoderTicksToInches(slaveData.getMotorCurrentPosition(1)));
        wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(0)));
        wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(1)));
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        RevBulkData masterData = master.getBulkInputData();
        RevBulkData slaveData = slave.getBulkInputData();

        if (masterData == null || slaveData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelVelo = new ArrayList<>();

        wheelVelo.add(encoderTicksToInches(slaveData.getMotorVelocity(0)));
        wheelVelo.add(encoderTicksToInches(slaveData.getMotorVelocity(1)));
        wheelVelo.add(encoderTicksToInches(masterData.getMotorVelocity(0)));
        wheelVelo.add(encoderTicksToInches(masterData.getMotorVelocity(1)));
        return wheelVelo;
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
}