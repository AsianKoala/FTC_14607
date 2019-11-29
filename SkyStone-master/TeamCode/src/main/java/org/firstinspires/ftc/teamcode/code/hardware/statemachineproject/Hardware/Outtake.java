package org.firstinspires.ftc.teamcode.code.hardware.statemachineproject.Hardware;

import org.firstinspires.ftc.teamcode.code.hardware.statemachineproject.HelperClasses.Firefly;
import org.openftc.revextensions2.ExpansionHubServo;
import static org.firstinspires.ftc.teamcode.code.GLOBALS.*;


public class Outtake {

    private ExpansionHubServo rotater;
    private ExpansionHubServo flipper;
    private ExpansionHubServo gripper;
    private ExpansionHubServo leftSlam;
    private ExpansionHubServo rightSlam;
    private Firefly robot;
    private boolean isDebugging = false;

    public Outtake(Firefly robot, ExpansionHubServo rotater, ExpansionHubServo flipper, ExpansionHubServo gripper, ExpansionHubServo leftSlam, ExpansionHubServo rightSlam) {
        this.rotater = flipper;
        this.flipper = flipper;
        this.gripper = gripper;
        this.leftSlam = leftSlam;
        this.rightSlam = rightSlam;
        this.robot = robot;
    }



    double flipperPosition;
    double gripperPosition;
    double rotaterPosition;
    double leftSlamPosition;
    double rightSlamPosition;



    /**
     * foundation movement controls
     */

    public void grabFoundation() {
        leftSlamPosition = 0.9;
        rightSlamPosition = 0.1;
    }

    public void ungrabFoundation() {
        leftSlamPosition = 0.1;
        rightSlamPosition = 0.9;
    }


    /**
     * flipper movement controls
     */

    public void flipOut() {
        flipperPosition = flipperOut;
    }

    public void flipReady() {flipperPosition = flipperHome;}

    public void flipMid() { flipperPosition = flipperBetween;}


    /**
     * gripper controls
     */
    public void grip() {gripperPosition = gripperGrip;}

    public void gripReady() {
        gripperPosition = gripperHome;
    }

    /**
     * rotater movement controls
     */

    public void rotaterOut() {
        rotaterPosition = rotaterOut;
    }

    public void rotaterReady() {
        rotaterPosition = rotaterHome;
    }



    public void setDebugging(boolean debugging) {
        isDebugging = debugging;
    }



    public void init() {
        rotaterReady();
        flipReady();
        gripReady();
    }

    public void update() {
        gripper.setPosition(gripperPosition);
        rotater.setPosition(rotaterPosition);
        flipper.setPosition(flipperPosition);



        robot.addSpace();

        robot.telemetry.addLine("------ outtake telem ---------");
        robot.telemetry.addData("gripper position", gripper.getPosition());
        robot.telemetry.addData("rotater position", rotater.getPosition());
        robot.telemetry.addData("flipper position", flipper.getPosition());

        if(isDebugging) {
            robot.telemetry.addData("gripper target position", gripperPosition);
            robot.telemetry.addData("rotater target position", rotaterPosition);
            robot.telemetry.addData("flipper target position", flipperPosition);
        }


    }

}
