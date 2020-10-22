package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.BaseOpMode;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.util.Globals.*;

public class DriveTrain extends Hardware {

    private ExpansionHubMotor leftFront, rightFront, leftBack, rightBack;
    private ExpansionHubMotor[] motors;

    public DriveTrain(ExpansionHubMotor leftFront, ExpansionHubMotor rightFront, ExpansionHubMotor  leftBack, ExpansionHubMotor rightBack) {

        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;

        motors = new ExpansionHubMotor[]{leftFront, leftBack, rightFront, rightBack};

        for(ExpansionHubMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void applyMovement() {
        // remember that motors on the right side are flipped (or maybe left i forgot)
        // https://www.vexforum.com/uploads/default/original/2X/c/c7963dec531b7e893ddcf91c4fc9159928c4b56c.jpeg
        double rawLeftFront = movementY + movementX + movementTurn;
        double rawLeftBack = movementY - movementX + movementTurn;
        double rawRightFront = - (movementY - movementX - movementTurn);
        double rawRightBack = - (movementY + movementX - movementTurn);
        double[] powers = {rawLeftFront, rawLeftBack, rawRightFront, rawRightBack};

        // if its over 1 then lower everything by a similar ratio to keep the same profile
        double biggestAbsPower = Math.abs(rawLeftFront);
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

    @Override
    public void debugUpdate() {
        for(int i=0; i<3; i++)
            parentOpMode.telemetry.addData(motors[i] + " amps drawn", motors[i].getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        for(int i=0; i<3; i++)
            parentOpMode.telemetry.addData(motors[i] + " connection info", motors[i].getConnectionInfo());
    }

    @Override
    public void update() {
        applyMovement();

        for(int i=0; i<3; i++)
            parentOpMode.telemetry.addData(motors[i] + " power", motors[i].getPower());

        if(isDebugging)
            debugUpdate();
    }

}
