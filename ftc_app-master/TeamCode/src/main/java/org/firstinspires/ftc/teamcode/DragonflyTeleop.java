package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//test
@TeleOp(name="Teleop : Dragonfly", group="Dragonfly")
public class DragonflyTeleop extends OpMode{


    HardwareDragonfly robot = new HardwareDragonfly();
    boolean closed = false;
    int close_count = 0;
    double speed_multiplier = 1;
    int lift_motor_position;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.resetEncoders();
        lift_motor_position = robot.lift.getCurrentPosition();
        telemetry.addData("Say", "Hello Driver");
        updateTelemetry(telemetry);
    }

    public void init_loop() {
        telemetry.addData("status", "loop test... waiting for start");
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    @Override
    public void loop() {

        double threshold = 0.1;

        telemetry.addData("gamepad1 LX", gamepad1.left_stick_x);
        telemetry.addData("gamepad1 LY", gamepad1.left_stick_y);
        telemetry.addData("gamepad1 RX", gamepad1.right_stick_x);
        telemetry.addData("gamepad1 RY", gamepad1.right_stick_y);
        telemetry.addData("lift ENCODER", robot.lift.getCurrentPosition());
        telemetry.addData("lift target position var", lift_motor_position);
        telemetry.addData("lift actual target", robot.lift.getTargetPosition());
        updateTelemetry(telemetry);
//
//        if(gamepad1.a){
//            robot.turnServoContinuous(1);
//        }else if(gamepad1.y){
//            robot.turnServoContinuous(-1);
//        }else{
//            robot.stopServoContinuous();
//        }

        double leftDrivePower = gamepad2.left_stick_y;
        double rightDrivePower = gamepad2.right_stick_y;
        if(Math.abs(leftDrivePower) < 0.05) leftDrivePower = 0;
        if(Math.abs(rightDrivePower) < 0.05) rightDrivePower = 0;
        robot.driveLimitless(leftDrivePower, rightDrivePower);

        robot.intake.setPower(gamepad1.right_stick_y);

        double armPower = gamepad1.left_trigger-gamepad1.right_trigger;
        if(Math.abs(armPower)<0.05) armPower = 0;
        robot.arm.setPower(armPower);

        double cascadePower = 0;
        if(gamepad1.left_bumper) cascadePower = 0.5;
        if(gamepad1.right_bumper) cascadePower = -0.5;
        robot.cascade.setPower(cascadePower);

        double liftPower = gamepad2.left_stick_y;
        if(Math.abs(liftPower)<0.1){
            liftPower = 0;
        }else {
            lift_motor_position += (int) (10 * liftPower);
        }
        //robot.lift.setTargetPosition(lift_motor_position);
        robot.lift.setPower(-Math.max(-1.0, Math.min((lift_motor_position-robot.lift.getCurrentPosition())/150, 1.0)));

        //debug
//        telemetry.addData("count ", close_count);
//        telemetry.update();
//
//
//        if(gamepad2.left_stick_button || gamepad2.right_stick_button){
//            speed_multiplier = 1;
//        }else{
//            speed_multiplier = 0.5;
//        }
//
//        if(gamepad2.left_stick_y == 0 && gamepad2.left_stick_x == 0 && !(Math.abs(gamepad2.right_stick_x) > threshold))
//        {
//            robot.fr.setPower(0);
//            robot.fl.setPower(0);
//            robot.br.setPower(0);
//            robot.bl.setPower(0);
//        }
//        else// if(Math.abs(gamepad2.left_stick_y) > threshold || Math.abs(gamepad2.left_stick_x) > threshold)
//        {
//            robot.fl.setPower(speed_multiplier *( (gamepad2.left_stick_y - gamepad2.left_stick_x)/2-(gamepad2.right_stick_x)/2));
//            robot.bl.setPower(speed_multiplier *( (gamepad2.left_stick_y + gamepad2.left_stick_x)/2-(gamepad2.right_stick_x)/2));
//            robot.fr.setPower(speed_multiplier *( (-gamepad2.left_stick_y - gamepad2.left_stick_x)/2-(gamepad2.right_stick_x)/2));
//            robot.br.setPower(speed_multiplier *( (-gamepad2.left_stick_y + gamepad2.left_stick_x)/2-(gamepad2.right_stick_x)/2));
//        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("stopping", 0);
        telemetry.update();
    }

}
