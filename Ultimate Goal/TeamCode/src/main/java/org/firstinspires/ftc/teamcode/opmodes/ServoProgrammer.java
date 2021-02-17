package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.Robot;
import org.openftc.revextensions2.ExpansionHubServo;

@TeleOp(name = "servo prgrm")
public class ServoProgrammer extends Robot {
    public double leftPivot = 0.5;
    public double rightPivot = 0.5;
    public ExpansionHubServo leftPivotServo, rightPivotServo;
    // 0.6  left grabber closed
    // 0.25 right grabber closed
    // 0.25 left grabber open
    // 0.6  right grabber open
    // 0.7  left pivot out
    // 0.1  right pivot out
    // 0.0  left pivot in
    // 0.8  right pivot in

    // 0.8 right in
    // 0.15 right out
    // 0.65 left out
    // 0.0 left in

    @Override
    public void init() {
        super.init();
        leftPivotServo = hardwareMap.get(ExpansionHubServo.class, "leftPivot");
        rightPivotServo = hardwareMap.get(ExpansionHubServo.class, "rightPivot");
    }

    @Override
    public void loop() {
        super.loop();
        try {
            controlServo();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        telemetry.addLine("leftPivot: " + leftPivot);
        telemetry.addLine("rightPivot: " + rightPivot);
    }

    public void controlServo() throws InterruptedException {
        if(gamepad1.left_bumper) {
            leftPivot += 0.05;
            sleep();
        }
        if(gamepad1.left_trigger > 0.7) {
            leftPivot -= 0.05;
            sleep();
        }
        if(gamepad1.right_bumper) {
            rightPivot += 0.05;
            sleep();
        }
        if(gamepad1.right_trigger > 0.7) {
            rightPivot -= 0.05;
            sleep();
        }
        if(gamepad1.x) {
            leftPivotServo.setPosition(leftPivot);
        }
        if(gamepad1.b) {
            rightPivotServo.setPosition(rightPivot);
        }
    }

    void sleep() throws InterruptedException {
        Thread.sleep(500);
    }
}
