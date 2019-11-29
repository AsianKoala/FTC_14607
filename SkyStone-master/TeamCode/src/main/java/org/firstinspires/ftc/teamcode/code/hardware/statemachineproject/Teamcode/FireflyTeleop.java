package org.firstinspires.ftc.teamcode.code.hardware.statemachineproject.Teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.code.hardware.statemachineproject.HelperClasses.Firefly;
import org.firstinspires.ftc.teamcode.code.hardware.statemachineproject.HelperClasses.TimeProfiler;

@TeleOp(name = "new test teleop", group = "teleop")
public class FireflyTeleop extends Firefly {


    @Override
    public void init() {
        super.init();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
    }


    private TimeProfiler tp1 = new TimeProfiler(1000);
    private TimeProfiler tp2 = new TimeProfiler(1000);
    private TimeProfiler tp3 = new TimeProfiler(1000);
    private TimeProfiler tp4 = new TimeProfiler(1000);
    private TimeProfiler tp5 = new TimeProfiler(1000);

    @Override
    public void loop() {


        tp1.markStart();
        teleopDrivetrainControl(); // super method (myDrivetrain is private)
        tp1.markEnd();


    }




    /**
     * our slide control
     */
    private void slideControl() {

        double increment = gamepad2.right_stick_y * 100;

        if(Math.abs(increment) > 25) { mySlide.manualMovement(increment, true); }
        if(gamepad2.x) { mySlide.goPsuedohome(); }
        if(gamepad2.b) { mySlide.goHome(); }

    }





    /**
     * our intake control
     */
    private void intakeControl() {

        double leftIntakePower = gamepad2.left_stick_y - gamepad2.left_stick_x;
        double rightIntakePower = gamepad2.left_stick_y + gamepad2.left_stick_x;

        myIntake.manualControl(leftIntakePower > 0.1 ? 0.5 * -leftIntakePower : 0,
                rightIntakePower > 0.1 ? 0.5 * -rightIntakePower : 0);

    }

}
