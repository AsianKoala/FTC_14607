package org.firstinspires.ftc.teamcode.roadrunner.teamcode.drive.mecanum;


import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

public class Intake {
    private HouseFly robot;
    private ExpansionHubMotor leftIntake;
    private ExpansionHubMotor rightIntake;
    private ExpansionHubServo leftSlam;
    private ExpansionHubServo rightSlam;

    public Intake(HouseFly robot, ExpansionHubMotor leftIntake, ExpansionHubMotor rightIntake, ExpansionHubServo leftSlam, ExpansionHubServo rightSlam) {

        this.robot = robot;
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;
        this.leftSlam = leftSlam;
        this.rightSlam = rightSlam;

        // set intake power 0
        // limit servo movement


    }

    private INTAKE_STATES intakeStates = INTAKE_STATES.OFF;

    private enum INTAKE_STATES {
        IN,
        OUT,
        OFF
    }


    private double intakeMotorCurrentPower = 0.0;

    private double getCurrentIntakePower() { return intakeMotorCurrentPower;}

    private void setIntakePower(double power) {
        intakeMotorCurrentPower = power;
    }


    private void handleMotorPower() {
        if(intakeStates == INTAKE_STATES.IN) {
            intakeMotorCurrentPower = 1;
        }

        if(intakeStates == INTAKE_STATES.OUT) {
            intakeMotorCurrentPower = -1;
        }

        if(intakeStates == INTAKE_STATES.OFF) {
            intakeMotorCurrentPower = 0;
        }
    }


    /**
     * Allow outside control of intake
     */
    public void turnOffIntake() { intakeStates = INTAKE_STATES.OFF;}
    public void turnOnIntake() { intakeStates = INTAKE_STATES.IN;}
    public void turnOnReverseIntake() { intakeStates = INTAKE_STATES.OUT;}


}
