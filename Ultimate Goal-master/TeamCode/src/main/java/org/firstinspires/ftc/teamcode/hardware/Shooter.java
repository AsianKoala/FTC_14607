package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.openftc.revextensions2.ExpansionHubMotor;

public class Shooter extends Hardware {
    private final ExpansionHubMotor leftShooter, rightShooter;
    public Shooter(ExpansionHubMotor leftShooter, ExpansionHubMotor rightShooter) {
        this.leftShooter = leftShooter;
        this.rightShooter = rightShooter;
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        Hardware.allHardwareComponents.add(this);
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
        leftShooter.setPower(power);
        rightShooter.setPower(power);
    }

    @Override
    public void update() {

    }
}
