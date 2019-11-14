package org.firstinspires.ftc.teamcode.roadrunner.teamcode.drive.mecanum;

import org.firstinspires.ftc.teamcode.revextensions2.ExpansionHubServo;

public class FoundationMover {
    private ExpansionHubServo leftFoundationMover;
    private ExpansionHubServo rightFoundationMover;
    private DriveBase robot;

    public FoundationMover(DriveBase robot, ExpansionHubServo leftFoundationMover, ExpansionHubServo rightFoundationMover) {
        this.robot = robot;
        this.leftFoundationMover = leftFoundationMover;
        this.rightFoundationMover = rightFoundationMover;

        // limit servo movement
        // TODO: CHANGE THESE VALUES TO ACTUAL VALUES
        leftFoundationMover.scaleRange(90,180);
        rightFoundationMover.scaleRange(0,90);

        // set servo pos to ready position
    }

    public void slam() {
        leftFoundationMover.setPosition(1);
        rightFoundationMover.setPosition(1);
    }

    public void ready() {
        leftFoundationMover.setPosition(0);
        rightFoundationMover.setPosition(0);
    }
}
