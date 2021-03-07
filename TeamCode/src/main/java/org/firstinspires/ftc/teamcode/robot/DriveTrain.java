package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmodes.Robot;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;


public class DriveTrain extends Hardware {
    public ExpansionHubMotor frontLeft, frontRight, backLeft, backRight;
    public ExpansionHubMotor[] allMotors;

    public DriveTrain(ExpansionHubMotor frontLeft, ExpansionHubMotor frontRight, ExpansionHubMotor backLeft, ExpansionHubMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        allMotors = new ExpansionHubMotor[]{frontLeft, frontRight, backLeft, backRight};

        for(ExpansionHubMotor e : allMotors) {
            e.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            e.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            e.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    @Override
    public void update(Robot robot) {
        double rawFL = -robot.currDrivePowers.y + robot.currDrivePowers.x + robot.currDrivePowers.heading;
        double rawFR = -robot.currDrivePowers.y + robot.currDrivePowers.x - robot.currDrivePowers.heading;
        double rawBL = -robot.currDrivePowers.y - robot.currDrivePowers.x + robot.currDrivePowers.heading;
        double rawBR = -robot.currDrivePowers.y - robot.currDrivePowers.x - robot.currDrivePowers.heading;
        List<Double> powers = Arrays.asList(rawFL, rawFR, rawBL, rawBR);

        double absMax = Math.max(Math.abs(Collections.max(powers)), Math.abs(Collections.min(powers)));
        if(absMax > 1) {
            for(int i=0; i<powers.size(); i++)
                powers.set(i, powers.get(i) / absMax);
        }

        for(int i=0; i<powers.size(); i++) {
            allMotors[i].setPower(powers.get(i));
        }
    }
}
