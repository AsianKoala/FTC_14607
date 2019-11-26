package org.firstinspires.ftc.teamcode.code.Hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.openftc.revextensions2.ExpansionHubMotor;
import java.util.ArrayList;

public class DriveTrain {
    public ExpansionHubMotor frontLeft;
    public ExpansionHubMotor frontRight;
    public ExpansionHubMotor backLeft;
    public ExpansionHubMotor backRight;
    public ArrayList<ExpansionHubMotor> allMotors = new ArrayList<>();

    public DriveTrain(ExpansionHubMotor frontLeft, ExpansionHubMotor frontRight, ExpansionHubMotor backLeft, ExpansionHubMotor backRight, ExpansionHubMotor.RunMode runmode) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        allMotors.add(frontLeft);
        allMotors.add(frontRight);
        allMotors.add(backLeft);
        allMotors.add(backRight);

        for(ExpansionHubMotor expansionHubMotor : allMotors) {
            expansionHubMotor.setMode(runmode);
            expansionHubMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if(runmode == DcMotor.RunMode.RUN_TO_POSITION) {
                expansionHubMotor.setTargetPosition(0);
            }
        }
    }


    public void driveMecanum(double xPower,double yPower,double zPower) {
        frontRight.setPower(1 * (yPower + xPower + -zPower));
        backLeft.setPower(1 * (yPower + -xPower + zPower));
        frontLeft.setPower(1 * (yPower + xPower+ zPower));
        backRight.setPower(1 * (yPower + -xPower + -zPower));
    }

    public void driveMecanum(Pose2d pose, double powerMultiplier) {

    }

}
