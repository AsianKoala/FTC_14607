package org.firstinspires.ftc.teamcode.code.hardware.statemachineproject.Hardware;

import android.annotation.SuppressLint;
import android.support.annotation.NonNull;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.code.hardware.statemachineproject.HelperClasses.Firefly;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.util.LynxOptimizedI2cFactory;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import static org.firstinspires.ftc.teamcode.code.GLOBALS.*;

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
    private boolean isDebugging = false;
    public ArrayList<ExpansionHubMotor> allMotors = new ArrayList<>();
    
    private Firefly robot;


    public DriveTrain(Firefly robot, ExpansionHubMotor frontLeft, ExpansionHubMotor frontRight, ExpansionHubMotor backLeft, ExpansionHubMotor backRight, ExpansionHubEx master, ExpansionHubEx slave, BNO055IMU imu) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.master = master;
        this.slave = slave;
        this.imu = imu;
        
        this.robot = robot;

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
        double rawFR = yPower + xPower - turnPower;
        double rawBL = yPower + -xPower + turnPower;
        double rawFL = yPower + xPower+ turnPower;
        double rawBR = yPower + -xPower + -turnPower;

        ArrayList<Double> allPowers = new ArrayList<>();
        allPowers.add(rawFL);
        allPowers.add(rawFR);
        allPowers.add(rawBL);
        allPowers.add(rawBR);

        double scaleAmt;
        double biggestPower = rawFL;

        // see if biggest power is > 1 for any reason
        for(Double currPower : allPowers) {
            if(biggestPower > 1.0) {
                biggestPower = currPower;
                scaleAmt = 1.0/biggestPower;
            }
        }

        int i = 0;
        for(ExpansionHubMotor motor : allMotors) {
            motor.setPower(allPowers.get(i));
            i++;
        }


    }
    


    public void setDebugging(boolean debugging) {
        isDebugging = debugging;
    }


 @SuppressLint("DefaultLocale")
    private String mecanumPowers() {
        return String.format(
                "\n" +
                        "(%.1f)---(%.1f)\n" +
                        "|   Front   |\n" +
                        "|           |\n" +
                        "|           |\n" +
                        "(%.1f)---(%.1f)\n"
                , frontLeft.getPower(), frontRight.getPower(), backLeft.getPower(), backRight.getPower());
    }



    /**
     * so we dont actually need to update/apply movement for our drivetrain since roadrunner takes care of that (?)
     * but we do for teleop so slap this shit in here
     */

    public void update() {
        driveMecanum(movementX, movementY, movementTurn); // the robot will only move if we change movementX, movementY, or movementTurn

        robot.addSpace();
        robot.telemetry.addLine("-------- drivetrain telem ---------");
        if (isDebugging) {
            robot.telemetry.addLine(mecanumPowers());
            robot.telemetry.addData("movement x var", movementX);
            robot.telemetry.addData("movement y var", movementY);
            robot.telemetry.addData("movement turn var", movementTurn);
        }
    }
}
