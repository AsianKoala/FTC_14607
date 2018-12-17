package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

//test
@TeleOp(name="Teleop : Dragonfly", group="Dragonfly")
public class DragonflyTeleop extends OpMode{

    final int ARM_MAX_ENCODER = 999;
    final int CASCADE_MAX_ENCODER = 999;
    final int LIFT_MAX_ENCODER = 999;

    final int ARM_ENCODER_POSITION_HORIZONTAL = 555;
    final int ARM_ENCODER_POSITION_NEUTRAL_EXTENDED = 555;
    final int ARM_ENCODER_POSITION_VERTICAL = 555;


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
        updateTelemetry(telemetry);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    public static double expo(double driveVal){
        if (driveVal>0) {
            return (0.3*((121.3416 + (0.301205 - 121.3416)/(1 + Math.pow(100*driveVal/56.03234, 2.711895)))-0.301))/100;//0.5*(0.0004*(Math.pow((driveVal-50), 3))+50);//Math.pow(2, driveVal / 15.02) - 1;
        }else {
            return -(0.3*((121.3416 + (0.301205 - 121.3416)/(1 + Math.pow(-100*driveVal/56.03234, 2.711895)))-0.301))/100;//-0.5*(0.0004*(Math.pow((-driveVal-50), 3))+50);
        }
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
        telemetry.addData("cascade ENCODER", robot.cascade.getCurrentPosition());
        telemetry.addData("arm ENCODER", robot.arm.getCurrentPosition());
        telemetry.addData("fl ENCODER", robot.fl.getCurrentPosition());
        telemetry.addData("fr ENCODER", robot.fr.getCurrentPosition());

        telemetry.addData("IMU", robot.getHeading());


        double leftDrivePower = expo(gamepad2.left_stick_y);
        double rightDrivePower = expo(gamepad2.right_stick_y);
        if(Math.abs(leftDrivePower) < 0.05) leftDrivePower = 0;
        if(Math.abs(rightDrivePower) < 0.05) rightDrivePower = 0;
//        if(gamepad2.right_trigger>0.5){
//            leftDrivePower*=3;
//            rightDrivePower*=3;
//        }
//        if(gamepad2.left_trigger>0.5){
//            leftDrivePower=gamepad2.left_stick_y/7; //5???
//            rightDrivePower=gamepad2.right_stick_y/7;
//        }

        if(gamepad2.right_trigger > 0.05){
            leftDrivePower = gamepad2.right_trigger/4;
            rightDrivePower = gamepad2.right_trigger/4;
        }

        if(gamepad2.left_trigger > 0.05){
            leftDrivePower = -gamepad2.left_trigger/4;
            rightDrivePower = -gamepad2.left_trigger/4;
        }

//        leftDrivePower = leftDrivePower/8;
//        rightDrivePower = rightDrivePower/8;
        if(gamepad2.left_bumper){
        }
        if(gamepad2.right_bumper){
            leftDrivePower = leftDrivePower*4;
            rightDrivePower = rightDrivePower*4;
        }

        robot.driveLimitless((leftDrivePower), (rightDrivePower));


        telemetry.addData("leftDrivePower", -expo(leftDrivePower));
        telemetry.addData("rightDrivePower", expo(rightDrivePower));
        telemetry.addData("lift motor runmode", robot.lift.getZeroPowerBehavior());
        telemetry.addData("intake power get", robot.intake.getPower());
        telemetry.addData("intake2 power get", robot.intake2.getPower());
        telemetry.addData("intakeDoor position get", robot.intakeDoor.getPosition());
        telemetry.addData("markerDeployer position get", robot.markerDeployer.getPosition());
        telemetry.addData("hangRelease position get", robot.hangRelease.getPosition());

        telemetry.addData("armpower", gamepad1.left_trigger-gamepad1.right_trigger);

        updateTelemetry(telemetry);

        double intakePower = gamepad1.right_stick_y;
        if(Math.abs(intakePower) < 0.05) intakePower = 0.5;
        robot.intake.setPower(-intakePower);
        robot.intake2.setPower(intakePower);
//        robot.intake.setPower(gamepad1.right_stick_y/2); //DEBUGGER
//        robot.intake2.setPower(gamepad1.right_stick_y/2);
        //intake power !!

        if(!gamepad1.dpad_down){
            robot.intakeDoor.setPosition(0);
        }else{
            robot.intakeDoor.setPosition(0.6);
        }

        if(gamepad1.y){
         }else{
            robot.hangRelease.setPosition(0);
        }

        if(gamepad1.b){
            robot.markerDeployer.setPosition(0.85);
        }else{
            robot.markerDeployer.setPosition(0);
        }

        if(gamepad1.a){
            robot.hookRelease.setPosition(0.13);
        }else{
            robot.hookRelease.setPosition(0.6);
        }

        double armPower = gamepad1.left_trigger-gamepad1.right_trigger;
        if(Math.abs(armPower)<0.05) armPower = 0;
        if(!gamepad1.x && robot.arm.getCurrentPosition() > 0 && armPower > 0) armPower = 0;
        if(!gamepad1.x && robot.arm.getCurrentPosition() < -1577 && armPower < 0) armPower = 0;
        robot.arm.setPower(armPower);

        double cascadePower = 0;
        if(gamepad1.left_bumper) cascadePower = 0.5;
        if(gamepad1.right_bumper) cascadePower = -0.5;
        if(!gamepad1.x && robot.cascade.getCurrentPosition() > 0 && cascadePower < 0) cascadePower = 0;
        if(!gamepad1.x && robot.cascade.getCurrentPosition() < -3310 && cascadePower > 0) cascadePower = 0;
        robot.cascade.setPower(cascadePower);

        double liftPower = -gamepad1.left_stick_y;
        if(Math.abs(liftPower)<0.1){
            liftPower = 0;
            robot.lift.setPower(0);
        }else {
            if(!gamepad1.x && robot.lift.getCurrentPosition() > 0 && liftPower < 0) liftPower = 0;
            if(!gamepad1.x && robot.lift.getCurrentPosition() < -4684 && liftPower > 0) liftPower = 0;
//            lift_motor_position += (int) (10 * liftPower);
            robot.lift.setPower(liftPower);
        }
//        robot.lift.setPower(-Math.max(-1.0, Math.min((lift_motor_position-robot.lift.getCurrentPosition())/150, 1.0)));

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
