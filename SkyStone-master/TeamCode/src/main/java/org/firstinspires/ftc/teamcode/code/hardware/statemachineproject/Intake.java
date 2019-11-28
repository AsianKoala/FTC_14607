package org.firstinspires.ftc.teamcode.code.hardware.statemachineproject;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.ArrayList;

public class Intake {
    private ExpansionHubMotor leftIntake;
    private ExpansionHubMotor rightIntake;
    public ArrayList<ExpansionHubMotor> allMotors = new ArrayList<>();

    private final double maxSpeed = 0.75;
    private double leftIntakeSpeed = 0;
    private double rightIntakeSpeed = 0;

    INTAKE_POWERS lastIntakePower = INTAKE_POWERS.OFF;
    INTAKE_POWERS intakePowers = INTAKE_POWERS.OFF;

    enum INTAKE_POWERS {
        ON,
        OFF,
        REVERSE,
        CUSTOM
    }

    public Intake(ExpansionHubMotor leftIntake, ExpansionHubMotor rightIntake) {
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;
        allMotors.add(leftIntake);
        allMotors.add(rightIntake);

        for(ExpansionHubMotor expansionHubMotor : allMotors) {
            expansionHubMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            expansionHubMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        intakePowers = INTAKE_POWERS.OFF;
    }




    private void setIntakePowers(double leftPower, double rightPower) {
        leftIntakeSpeed = leftPower;
        rightIntakeSpeed = rightPower;
    }


    public void turnOnIntake() { intakePowers = INTAKE_POWERS.ON; }
    public void turnOffIntake() { intakePowers = INTAKE_POWERS.OFF; }
    public void reverseIntake() { intakePowers = INTAKE_POWERS.REVERSE; }
    public void manualControl(double customLeftIntakeSpeed, double customRightIntakeSpeed) {
        intakePowers = INTAKE_POWERS.CUSTOM;
        leftIntakeSpeed = customLeftIntakeSpeed;
        rightIntakeSpeed = customRightIntakeSpeed;
    }


    private void HandlePower() {
        if(intakePowers == INTAKE_POWERS.ON) {
            setIntakePowers(maxSpeed, maxSpeed);
        }

        if(intakePowers == INTAKE_POWERS.REVERSE) {
            setIntakePowers(-maxSpeed, -maxSpeed);
        }

        if(intakePowers == INTAKE_POWERS.OFF) {
            setIntakePowers(0,0);
        }

        if(intakePowers == INTAKE_POWERS.CUSTOM) {

        }
    }



    public void update() {
        // avoid spamming to make loop faster
        if(lastIntakePower != intakePowers) {
            HandlePower();
            leftIntake.setPower(leftIntakeSpeed);
            rightIntake.setPower(rightIntakeSpeed);

            lastIntakePower = intakePowers;
        }
    }


}
