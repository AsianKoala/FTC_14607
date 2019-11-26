package org.firstinspires.ftc.teamcode.code.Hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.ArrayList;

public class Intake {
    public ExpansionHubMotor leftIntake;
    public ExpansionHubMotor rightIntake;
    public ArrayList<ExpansionHubMotor> allMotors = new ArrayList<>();

    public Intake(ExpansionHubMotor leftIntake, ExpansionHubMotor rightIntake) {
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;
        allMotors.add(leftIntake);
        allMotors.add(rightIntake);

        for(ExpansionHubMotor expansionHubMotor : allMotors) {
            expansionHubMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            expansionHubMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }



    public void setIntakePowers(Pose2d pose) {

    }
}
