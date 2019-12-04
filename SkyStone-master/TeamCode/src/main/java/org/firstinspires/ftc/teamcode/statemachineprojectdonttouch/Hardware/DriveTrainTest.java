package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Auto.roadrunner.drive.mecanum.SampleMecanumDriveREV;
import org.openftc.revextensions2.ExpansionHubMotor;

public class DriveTrainTest  {

    public ExpansionHubMotor frontLeft, frontRight, backLeft, backRight;

    public DriveTrainTest(ExpansionHubMotor frontLeft, ExpansionHubMotor frontRight, ExpansionHubMotor backLeft, ExpansionHubMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void driveMecanum(double xPower,double yPower,double turnPower) {
        double rawFR = yPower + xPower - turnPower;
        double rawBL = yPower + -xPower + turnPower;
        double rawFL = yPower + xPower+ turnPower;
        double rawBR = yPower + -xPower + -turnPower;


        double scaleAmt = 1.0;
        double maxRaw = Math.abs(rawFR);
        if(Math.abs(rawBL) > maxRaw) { maxRaw = Math.abs(rawBL); }
        if(Math.abs(rawFL) > maxRaw) { maxRaw = Math.abs(rawFL); }
        if(Math.abs(rawBR) > maxRaw) { maxRaw = Math.abs(rawBR); }

        if(maxRaw > 1.0) {
            scaleAmt = 1.0/maxRaw;
        }

        rawFR *= scaleAmt;
        rawFL *= scaleAmt;
        rawBR *= scaleAmt;
        rawBL *= scaleAmt;


        frontLeft.setPower(rawFL);
        frontRight.setPower(rawFR);
        backLeft.setPower(rawBL);
        backRight.setPower(rawBR);

    }

}
