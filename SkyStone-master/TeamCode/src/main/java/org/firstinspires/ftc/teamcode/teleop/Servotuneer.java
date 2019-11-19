package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Servotuneer extends OpMode {

    private Servo flipper = null;
    double flipperPosition = 0.5;
    @Override
    public void init() {

        flipper = hardwareMap.get(Servo.class, "flipper");

    }

    @Override
    public void loop() {

        if(gamepad1.left_trigger > 0.25) flipperPosition += 0.01;

        if(gamepad1.right_trigger > 0.25) flipperPosition -= 0.01;


        flipper.setPosition(flipperPosition);
        try { Thread.sleep(25); }
        catch(Exception e) {}


        telemetry.addData("flipper pos", flipperPosition);
        telemetry.update();

    }



}
