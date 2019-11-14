package org.firstinspires.ftc.teamcode.roadrunner.teamcode.drive.mecanum;

import org.firstinspires.ftc.teamcode.revextensions2.ExpansionHubServo;

public class FoundationMover {
    private ExpansionHubServo leftFoundationMover;
    private ExpansionHubServo rightFoundationMover;
    private HouseFly robot;

    public FoundationMover(HouseFly robot, ExpansionHubServo leftFoundationMover, ExpansionHubServo rightFoundationMover) {
        this.robot = robot;
        this.leftFoundationMover = leftFoundationMover;
        this.rightFoundationMover = rightFoundationMover;

        // limit servo movement
        // TODO: CHANGE THESE VALUES TO ACTUAL VALUES
        leftFoundationMover.scaleRange(90,180);
        rightFoundationMover.scaleRange(90,0);


        // set servo pos to ready position
    }


    MOVER_STATES moverStates = MOVER_STATES.READY;

    private enum MOVER_STATES {
        READY,
        UNLOAD
    }

    private double servoPosition = 0.0;

    private void HandleMovement() {
        if(moverStates == MOVER_STATES.READY) {
            servoPosition = 1.0;
        }


    }



}
