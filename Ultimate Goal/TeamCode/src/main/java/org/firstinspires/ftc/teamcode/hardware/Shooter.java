package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.openftc.revextensions2.ExpansionHubMotor;

public class Shooter extends Hardware {
    private final ExpansionHubMotor leftShooter, rightShooter;
    private double leftShooterPower, rightShooterPower;
    public Shooter(ExpansionHubMotor leftShooter, ExpansionHubMotor rightShooter) {
        this.leftShooter = leftShooter;
        this.rightShooter = rightShooter;
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooterPower = 0;
        rightShooterPower = 0;
    }

    public void turnOn() {
        setPower(1);
    }

    public void turnOff() {
        setPower(0);
    }

    public void reverse() {
        setPower(-1);
    }

    private void setPower(double power) {
        leftShooterPower = power;
        rightShooterPower = power;
    }

    @Override
    public void update() {
        leftShooter.setPower(leftShooterPower);
        rightShooter.setPower(rightShooterPower);
    }
}
