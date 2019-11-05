package org.firstinspires.ftc.teamcode.lastMinutescuffedPP.Hardware;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake {
    // link to the robot
    private Robot myRobot;

    // / / / / / / s o m e  _ n u m b e r s / / / / / / ///
    private double COLLECTER_FORWARDS_SPEED = 1.0;
    private double COLLECTER_BACKWARDS_SPEED = -1.0;


    private RevMotor leftSuck;
    private RevMotor rightSuck;


    public Intake(Robot myRobot, RevMotor leftSuck, RevMotor rightSuck) {

        // reverse one of the motors TODO: TEST TO SEE WHICH ONE ACTUALLY NEEDS REVERSING


        rightSuck.setDirection(DcMotorSimple.Direction.REVERSE);

        // link the hardware up
        this.myRobot = myRobot;
        this.leftSuck = leftSuck;
        this.rightSuck = rightSuck;

        //
    }

    private double intakeMotorCurrentPower = 0.0;

    /**
     * gets current power assigned to intake
     */
    public double getIntakeMotorCurrentPower() { return intakeMotorCurrentPower;}

    /** set intake power */
    public void setIntakePower (double power) {
        intakeMotorCurrentPower = power;
    }


    public intakeStates intakeState = intakeStates.off;
    public enum intakeStates {
        off,
        forwards,
        reverse
    }

    /**
     * This is called every update and will set the roller power automatically
     */

    public void handleIntakeStates() {
        if(intakeState == intakeStates.off) {
            setIntakePower(0);
        }
        if(intakeState == intakeStates.forwards) {
            setIntakePower(COLLECTER_FORWARDS_SPEED);
        }
        if(intakeState == intakeStates.reverse) {
            setIntakePower(COLLECTER_BACKWARDS_SPEED);
        }
    }

    /** Turns on intake */
    public void turnOnIntake() { intakeState = intakeStates.forwards; }

    /** turns reverse intake */
    public void reverseIntake() { intakeState = intakeStates.reverse;}

    /** turns off intake */
    public void turnOffIntake() {
        intakeState = intakeStates.off;
    }

    private double intakeMotorLastPower = 0;
    private long lastUpdateTime = 0;

    public void update() {

    }


}
