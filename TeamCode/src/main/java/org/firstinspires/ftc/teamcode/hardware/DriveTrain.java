package org.firstinspires.ftc.teamcode.main.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.openftc.revextensions2.ExpansionHubMotor;


public class DriveTrain extends Hardware {

    // hahaha this setup is so fucked i just realized now that the motor array and the motorpower array dont match up
    public static double movementX, movementY, movementTurn;
    private final ExpansionHubMotor frontLeft, frontRight, backLeft, backRight;
    private final ExpansionHubMotor[] motors;

    public DriveTrain(ExpansionHubMotor FL, ExpansionHubMotor FR, ExpansionHubMotor BL, ExpansionHubMotor BR) {
        frontLeft = FL;
        frontRight = FR;
        backLeft = BL;
        backRight = BR;

        motors = new ExpansionHubMotor[]{frontLeft, frontRight, backLeft, backRight};
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        for (ExpansionHubMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public static void stopMovement() {
        movementX = 0;
        movementY = 0;
        movementTurn = 0;
    }

    public void update() {
        // remember that motors on the right side are flipped (or maybe left i forgot)
        // https://www.vexforum.com/uploads/default/original/2X/c/c7963dec531b7e893ddcf91c4fc9159928c4b56c.jpeg
        double rawFrontLeft = -movementY + movementX + movementTurn;
        double rawBackLeft = -movementY - movementX + movementTurn;
        double rawFrontRight = -movementY + movementX - movementTurn;
        double rawBackRight = -movementY - movementX - movementTurn;
        double[] powers = {rawFrontLeft, rawFrontRight, rawBackLeft, rawBackRight};

        // if its over 1 then lower everything by a similar ratio to keep the same profile
        double biggestAbsPower = Math.abs(rawFrontLeft);
        for (double power : powers) {
            if (Math.abs(power) > biggestAbsPower)
                biggestAbsPower = Math.abs(power);
        }

        // divide everything to <= 1
        if (biggestAbsPower > 1) {
            for (int i = 0; i < powers.length; i++)
                powers[i] = powers[i] / biggestAbsPower;
        }

        for (int i = 0; i < powers.length; i++) {
            motors[i].setPower(powers[i]);
        }
    }

    @Override
    public String toString() {
        return "movementX: " + movementX + " movementY: " + movementY + " movementTurn: " + movementTurn;
    }

    @Override
    public void turnOn() {

    }

    @Override
    public void turnOff() {

    }

    @Override
    public void reverse() {

    }

}