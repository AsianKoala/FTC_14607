package org.firstinspires.ftc.teamcode.roadrunner.teamcode.drive.Hardware;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.revextensions2.ExpansionHubMotor;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.drive.opmode.Housefly;

@Config
public class Intake {
    private Housefly robot;

    private double SUCK = 1.0;
    private double VOMIT = 1.0;

    private ExpansionHubMotor leftSuck;
    private ExpansionHubMotor rightSuck;

    public Intake(Housefly robot, ExpansionHubMotor leftSuck, ExpansionHubMotor rightSuck) {
        this.robot = robot;
        this.leftSuck = leftSuck;
        this.rightSuck = rightSuck;

        // set intake power 0

        // update
    }

    private double intakeCurrentPower = 0.0;


    /**
     * gets current power assignde to intake
     */
    public double getIntakeCurrentPower() { return intakeCurrentPower;}

    public void setIntakePower(double power) { intakeCurrentPower = power;}


    public intakeStates intakeState = intakeStates.off;
    public enum intakeStates {
        off,
        on,
        reverse
    }


    /**
     * called every update and will set intake power automatically
     */
    public void handleIntakeStates() {
        if(intakeState == intakeStates.off)
            setIntakePower(0);

        if(intakeState == intakeStates.on) {
            setIntakePower(SUCK);
        }

        if(intakeState == intakeStates.reverse)
            setIntakePower(VOMIT);
    }


    public void turnOnIntake() { intakeState = intakeStates.on;}
    public void turnOffIntake() { intakeState = intakeStates.off;}
    public void reverseIntake() { intakeState = intakeStates.reverse;}

    public void update() {
        handleIntakeStates();
        leftSuck.setPower(intakeCurrentPower);
        rightSuck.setPower(intakeCurrentPower * -1);
    }
















}
