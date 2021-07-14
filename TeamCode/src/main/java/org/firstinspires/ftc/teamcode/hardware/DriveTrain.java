package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.openftc.revextensions2.ExpansionHubMotor;


public class DriveTrain {

    public static double movementX, movementY, movementTurn;
    private final ExpansionHubMotor frontLeft, frontRight, backLeft, backRight;
    private final ExpansionHubMotor[] motors;

    public DriveTrain(ExpansionHubMotor FL, ExpansionHubMotor FR, ExpansionHubMotor  BL, ExpansionHubMotor BR) {
        frontLeft = FL;
        frontRight = FR;
        backLeft = BL;
        backRight = BR;

        motors = new ExpansionHubMotor[]{frontLeft, frontRight, backLeft, backRight};
        for(ExpansionHubMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }


    public void update() {
        // https://www.vexforum.com/uploads/default/original/2X/c/c7963dec531b7e893ddcf91c4fc9159928c4b56c.jpeg
        double rawFrontLeft = -movementY + movementX + movementTurn;
        double rawBackLeft = -movementY - movementX + movementTurn;
        double rawFrontRight =  -movementY + movementX - movementTurn;
        double rawBackRight =  -movementY - movementX - movementTurn;
        double[] powers = {rawFrontLeft, rawFrontRight, rawBackLeft, rawBackRight};

        double biggestAbsPower = Math.abs(rawFrontLeft);
        for(double power : powers) {
            if(Math.abs(power) > biggestAbsPower)
                biggestAbsPower = Math.abs(power);
        }

        if(biggestAbsPower > 1) {
            for(int i=0; i<powers.length; i++)
                powers[i] = powers[i] / biggestAbsPower;
        }

        for(int i=0; i<powers.length; i++) {
            motors[i].setPower(powers[i]);
        }
    }

    @Override
    public String toString() {
        return "movementX: " + movementX + " movementY: " + movementY + " movementTurn: " + movementTurn;
    }
}