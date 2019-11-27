package org.firstinspires.ftc.teamcode.code.Hardware;

import android.support.annotation.NonNull;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.util.LynxOptimizedI2cFactory;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.roadrunner.teamcode.drive.DriveConstants.encoderTicksToInches;

public class DriveTrain extends SampleMecanumDriveBase {
    public ExpansionHubEx master, slave;
    public ExpansionHubMotor frontLeft;
    public ExpansionHubMotor frontRight;
    public ExpansionHubMotor backLeft;
    public ExpansionHubMotor backRight;
    private BNO055IMU imu;
    public ArrayList<ExpansionHubMotor> allMotors = new ArrayList<>();



    public DriveTrain(ExpansionHubMotor frontLeft, ExpansionHubMotor frontRight, ExpansionHubMotor backLeft, ExpansionHubMotor backRight, ExpansionHubEx master, ExpansionHubEx slave, BNO055IMU imu) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.master = master;
        this.slave = slave;
        this.imu = imu;

        allMotors.add(frontLeft);
        allMotors.add(frontRight);
        allMotors.add(backLeft);
        allMotors.add(backRight);




        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(master.getStandardModule(), 0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        for(ExpansionHubMotor expansionHubMotor : allMotors) {
            expansionHubMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            expansionHubMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

    }



    /*
    required methods cuz of roadrunner
     */


    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = frontLeft.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }


    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (ExpansionHubMotor motor : allMotors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, 1
            ));
        }
    }


    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData = master.getBulkInputData();
        RevBulkData bulkData2 = slave.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelPositions = new ArrayList<>();


        wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(0)));
        wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(1)));
        wheelPositions.add(encoderTicksToInches(bulkData2.getMotorCurrentPosition(0)));
        wheelPositions.add(encoderTicksToInches(bulkData2.getMotorCurrentPosition(1)));
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
     *
     * @param xPower  x translation of robot; gamepad1.leftstick.x during teleop
     * @param yPower  y translation of robot; gamepad1.leftstick.y during teleop
     * @param turnPower  turn of robot; gamepad1.rightstick.x during teleop
     */
    public void driveMecanum(double xPower,double yPower,double turnPower) {
        frontRight.setPower(1 * (yPower + xPower + -turnPower));
        backLeft.setPower(1 * (yPower + -xPower + turnPower));
        frontLeft.setPower(1 * (yPower + xPower+ turnPower));
        backRight.setPower(1 * (yPower + -xPower + -turnPower));
    }

    /**
     * literally the same exact one but i wanted to add an extra parameter since im too lazy to change all of the implementations of this method
     * @param xPower
     * @param yPower
     * @param turnPower
     * @param teleopControl
     */
    public void driveMecanum(double xPower,double yPower,double turnPower, boolean teleopControl, double multiplier) {
        yPower *=1;
        frontRight.setPower(multiplier * (yPower + xPower + -turnPower));
        backLeft.setPower(multiplier * (yPower + -xPower + turnPower));
        frontLeft.setPower(multiplier * (yPower + xPower+ turnPower));
        backRight.setPower(multiplier * (yPower + -xPower + -turnPower));
    }

}
