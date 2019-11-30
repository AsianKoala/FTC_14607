package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses.Firefly;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.ArrayList;

public class Intake {
    private ExpansionHubMotor leftIntake;
    private ExpansionHubMotor rightIntake;
    public ArrayList<ExpansionHubMotor> allMotors = new ArrayList<>();
    private Firefly robot;


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

    public Intake(Firefly robot, ExpansionHubMotor leftIntake, ExpansionHubMotor rightIntake) {
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;
        this.robot = robot;
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


public boolean isDebugging;
    public void setDebugging(boolean debugging) {
        isDebugging = debugging;
    }
    public void update() {
        // avoid spamming to make loop faster
        if(lastIntakePower != intakePowers) {
            HandlePower();
            leftIntake.setPower(leftIntakeSpeed);
            rightIntake.setPower(rightIntakeSpeed);

            lastIntakePower = intakePowers;
        }

        robot.telemetry.addData("left intake power", leftIntake.getPower());
        robot.telemetry.addData("right intake power", rightIntake.getPower());

        if(isDebugging) {
            robot.telemetry.addData("intakePower status", intakePowers);
            robot.telemetry.addData("left intake target speed", leftIntakeSpeed);
            robot.telemetry.addData("right intake target speed", rightIntakeSpeed);
        }

    }




}
