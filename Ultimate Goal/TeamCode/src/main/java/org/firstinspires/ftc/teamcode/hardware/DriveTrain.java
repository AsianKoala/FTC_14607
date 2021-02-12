package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.jetbrains.annotations.NotNull;
import org.openftc.revextensions2.ExpansionHubMotor;


public class DriveTrain extends Hardware {

    // hahaha this setup is so fucked i just realized now that the motor array and the motorpower array dont match up
    public static double movementX, movementY, movementTurn;
    private final ExpansionHubMotor frontRight, frontLeft, backLeft, backRight;
    private final ExpansionHubMotor[] motors;

    public DriveTrain(ExpansionHubMotor FL, ExpansionHubMotor FR, ExpansionHubMotor  BL, ExpansionHubMotor BR) {
        frontLeft = FL;
        backLeft = BL;
        frontRight = FR;
        backRight = BR;

        motors = new ExpansionHubMotor[]{frontRight, backLeft, frontLeft, backRight};
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        for(ExpansionHubMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void applyMovement() {
        // remember that motors on the right side are flipped (or maybe left i forgot)
        // https://www.vexforum.com/uploads/default/original/2X/c/c7963dec531b7e893ddcf91c4fc9159928c4b56c.jpeg
        double rawFrontLeft = movementY - movementX - movementTurn;
        double rawBackLeft = movementY - movementX + movementTurn;
        double rawFrontRight =  -movementY - movementX + movementTurn;
        double rawBackRight =  -movementY - movementX - movementTurn;
        double[] powers = {rawFrontLeft, rawBackLeft, rawFrontRight, rawBackRight};

        // if its over 1 then lower everything by a similar ratio to keep the same profile
        double biggestAbsPower = Math.abs(rawFrontLeft);
        for(double power : powers) {
            if(Math.abs(power) > biggestAbsPower)
                biggestAbsPower = Math.abs(power);
        }

        // divide everything to <= 1
        if(biggestAbsPower > 1) {
            for(int i=0; i<powers.length; i++)
                powers[i] = powers[i] / biggestAbsPower;
        }

        for(int i=0; i<powers.length; i++) {
            motors[i].setPower(powers[i]);
        }
    }

    public static void stopMovement() {
        movementX = 0;
        movementY = 0;
        movementTurn = 0;
    }

    @NotNull
    @Override
    public String toString() {
        return "movementX: " + movementX + " movementY: " + movementY + " movementTurn: " + movementTurn;
    }

    @Override
    public void update() {
        applyMovement();
//        parentOpMode.telemetry.addData("frontLeft power", frontLeft.getPower());
//        parentOpMode.telemetry.addData("frontRight power", frontRight.getPower());
//        parentOpMode.telemetry.addData("backLeft power", backLeft.getPower());
//        parentOpMode.telemetry.addData("backRight power", backRight.getPower());
    }

}