package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.teleop.ServoGlobals.*;

@TeleOp
public class ServoTuner extends OpMode {

    private Servo flipper,gripper,outtake;

    public boolean wantRealtime=true;
    public boolean wantReadyPos = false;



    @Override
    public void init() {
        flipper = hardwareMap.get(Servo.class, "flipper");
        gripper = hardwareMap.get(Servo.class, "gripper");
        outtake = hardwareMap.get(Servo.class, "outtake");
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        /**
         * to see if we want real time updates or not
         * we might not want real time updates if a servo is in a really bad pos and we have to change it without damaging the servo ro something
         * IDK I DONT EVEN THINK WE NEED IT BUT LETS JUST PUT IT HERE FOR SAFETY
         * anyway, if we press a then we want real time feedback, but if we press b then we dont
         */
        if(gamepad1.a || gamepad2.a) {
            wantRealtime = true;
        }

        if(gamepad1.b || gamepad2.b) {
            wantRealtime = false;
        }


        /**
         * if we want real time updates we need to know whether or not the we want real time updates from the ready pos or the "On" pos (default on pos)
         */
        if(gamepad1.y) {
            wantReadyPos = true;
        }

        if(gamepad2.y) {
            wantReadyPos = false;
        }




        /**
         * so gamepad1 will be for the "ready" position of the servo
         * gamepad2 will be the "on" position of the servo
         * anything on the left will be for subtracting
         * right wil be for adding
         * 'yes
         */

        if(gamepad1.left_bumper) flipperFlipPosition -=5;
        if(gamepad1.right_bumper) flipperReadyPosition +=5;
        if(gamepad1.left_trigger > 0.25) gripperReadyPosition -=5;
        if(gamepad1.right_trigger > 0.25) gripperReadyPosition +=5;
        if(gamepad1.dpad_left) outtakeReadyPosition -= 5;
        if(gamepad1.dpad_right) outtakeReadyPosition +=5;


        if(gamepad1.left_bumper) flipperFlipPosition -=5;
        if(gamepad1.right_bumper) flipperFlipPosition +=5;
        if(gamepad1.left_trigger > 0.25) gripperGripPosition -=5;
        if(gamepad1.right_trigger > 0.25) gripperGripPosition +=5;
        if(gamepad1.dpad_left) outtakeOutPosition -= 5;
        if(gamepad1.dpad_right) outtakeOutPosition +=5;

















        // called every loop to update pos
        if(wantRealtime) {
            /**
             * decides whether or not we want to update the servos to ready pos or on pos
             */
            if(wantReadyPos) updateReady();
            else updateOn();
        }

        else {
            /**
             * only updates position when gamepad1.x (ready pos) or gamepad2.x (onpos) is pressed
             */
            if(gamepad1.x) {
                updateReady();
                wantReadyPos = true;
            }
            if(gamepad2.y) {
                updateOn();
                wantReadyPos = false;
            }
        }


        telemetry.addData("flipper ready position", flipperReadyPosition);
        telemetry.addData("gripper ready position", gripperReadyPosition);
        telemetry.addData("outtake ready position", outtakeReadyPosition);
        telemetry.addData("flipper flip position", flipperFlipPosition);
        telemetry.addData("gripper grip position", gripperGripPosition);
        telemetry.addData("outtake out position", outtakeOutPosition);
        telemetry.addData("want realtime", wantRealtime);
        telemetry.addData("want ready pos", wantReadyPos);
        telemetry.update();

    }

    public void updateReady() {
        flipper.setPosition(flipperReadyPosition);
        gripper.setPosition(gripperReadyPosition);
        outtake.setPosition(outtakeReadyPosition);
        sleep(250);
    }

    public void updateOn() {
        flipper.setPosition(flipperFlipPosition);
        gripper.setPosition(gripperGripPosition);
        outtake.setPosition(outtakeOutPosition);
        sleep(250);
    }


    /**
     * for some reason the sleep method is actually created in LinearOpMode and not inherited through OpMode ??????
     * well i mean it makes sense since OpMode is more for like state machines and running stuff in parallel & shit
     * or you can just run the whole auto in the start() {} method
     * def not what we're doing
     * dont worry it works
     * kappa
     * @param milliseconds
     */
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
