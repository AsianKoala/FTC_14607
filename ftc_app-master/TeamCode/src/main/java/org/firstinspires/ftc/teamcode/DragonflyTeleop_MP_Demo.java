package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//test
@TeleOp(name="Teleop : Dragonfly DEMO", group="Dragonfly")
public class DragonflyTeleop_MP_Demo extends OpMode{

    HardwareDragonflyMP robot = new HardwareDragonflyMP();
    boolean closed = false;
    int close_count = 0;
    double speed_multiplier = 1;
    double intakePower = 0;
    int lift_motor_position;

    boolean intakeSlowMode = false;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
//        robot.resetEncoders(); TODO: RESET ARM ENCODERS DEPENDING ON AUTO RUN
//        lift_motor_position = robot.lift.getCurrentPosition();
//        robot.arm.setTargetPosition(0); //arm stays at previous position
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

    //DRIVE EXPO
    public static double expo(double driveVal){
        if (driveVal>0) {
            return (0.75*((121.3416 + (0.301205 - 121.3416)/(1 + Math.pow(100*driveVal/56.03234, 2.711895)))-0.301))/100;//0.5*(0.0004*(Math.pow((driveVal-50), 3))+50);//Math.pow(2, driveVal / 15.02) - 1;
        }else {
            return -(0.75*((121.3416 + (0.301205 - 121.3416)/(1 + Math.pow(-100*driveVal/56.03234, 2.711895)))-0.301))/100;//-0.5*(0.0004*(Math.pow((-driveVal-50), 3))+50);
        }
    }
    //END DRIVE EXPO

    @Override
    public void loop() {

        if(gamepad1.start && gamepad1.back){
            robot.resetEncoders();
        }

        // START TELEMETRY
        telemetry.addData("gamepad1 LX", gamepad1.left_stick_x);
        telemetry.addData("gamepad1 LY", gamepad1.left_stick_y);
        telemetry.addData("gamepad1 RX", gamepad1.right_stick_x);
        telemetry.addData("gamepad1 RY", gamepad1.right_stick_y);
        telemetry.addData("lift ENCODER", robot.lift.getCurrentPosition());
//        telemetry.addData("lift target position var", lift_motor_position);
        telemetry.addData("lift actual target", robot.lift.getTargetPosition());
        telemetry.addData("cascade ENCODER", robot.cascade.getCurrentPosition());
        telemetry.addData("arm ENCODER", robot.arm.getCurrentPosition());
        telemetry.addData("fl ENCODER", robot.fl.getCurrentPosition());
        telemetry.addData("fr ENCODER", robot.fr.getCurrentPosition());
        telemetry.addData("bl ENCODER", robot.bl.getCurrentPosition());
        telemetry.addData("br ENCODER", robot.br.getCurrentPosition());
        telemetry.addData("IMU", robot.getHeading());

        //BREAKING
        telemetry.addData("current draw of lift: ", robot.lift.getCurrentDraw());
        //END

        //END TELEMETRY


        //START DRIVE CODE
        double forwardsPower = expo(gamepad2.left_stick_y)*.5;
        double turningPower = expo(gamepad2.left_stick_x)*.5;

        if(Math.abs(forwardsPower) <= 0.05 && Math.abs(turningPower) <= 0.05){
            robot.driveLimitlessTeleop((0), (0));
        }
        else if(forwardsPower >= 0) {
            robot.driveLimitlessTeleop(forwardsPower+turningPower, forwardsPower-turningPower);
        }
        else if(forwardsPower < 0.4) {
            robot.driveLimitlessTeleop(forwardsPower-turningPower, forwardsPower+turningPower);
        }


        //START ADDITIONAL TELEMETRY
        telemetry.addData("lift motor runmode", robot.lift.getZeroPowerBehavior());
        telemetry.addData("intake power get", robot.intake.getPower());
        telemetry.addData("intake2 power get", robot.intake2.getPower());
        telemetry.addData("intakeDoor position get", robot.intakeDoor.getPosition());
        telemetry.addData("markerDeployer position get", robot.markerDeployer.getPosition());
        telemetry.addData("hangRelease position get", robot.hangRelease.getPosition());
        telemetry.addData("armpower", gamepad1.left_trigger-gamepad1.right_trigger);
        updateTelemetry(telemetry);
        //END ADDITIONAL TELEMETRY


        //START INTAKE

        if(gamepad1.right_stick_button){
            intakePower = 0;
        }
        if(Math.abs(gamepad1.right_stick_y) > 0.05){
            intakePower = gamepad1.right_stick_y;
            intakeSlowMode = false;
        }
//        intakePower = gamepad1.right_stick_y*0.85;
        if(Math.abs(gamepad1.right_stick_y) < 0.05 && intakePower!=0) intakePower = 0.85; //AUTOMATIC INTAKING

        if(intakeSlowMode == true){
            intakePower*=0.55; //experimental
        }
//        robot.intake.setPower(-intakePower);
        robot.intake_motor.setPower(-intakePower);

//        robot.intake.setPower(gamepad1.right_stick_y/2); //DEBUGGING USE ONLY
//        robot.intake2.setPower(gamepad1.right_stick_y/2);
//        if(gamepad1.right_bumper){ //default position
//            robot.intakeDoor.setPosition(0.6);
//        }
//        if(gamepad1.left_bumper){
//            robot.intakeDoor.setPosition(0);
//        }
        //END INTAKE


        //START ARM
        double armPower = gamepad1.left_trigger-gamepad1.right_trigger;
        if(Math.abs(armPower)<0.05) armPower = 0;
//        if(!gamepad1.dpad_up && robot.arm.getCurrentPosition() > 0 && armPower > 0) armPower = 0;
//        if(!gamepad1.dpad_up && robot.arm.getCurrentPosition() < -1577 && armPower < 0) armPower = 0;
        if(robot.arm.isBusy()) {

//
            robot.arm.setPower(Math.max((Math.abs(robot.arm.getCurrentPosition() - robot.arm.getTargetPosition())) / 100 * (0.2), (0.2)));
//            robot.arm.setPower(Math.max((Math.abs(robot.arm.getCurrentPosition() - robot.arm.getTargetPosition())) / 100 * (0.2), (0.2)));

//            robot.arm.setVelocity(200, AngleUnit.DEGREES);


        }else{
            robot.arm.setPower(0);
//            robot.arm.setVelocity(0, AngleUnit.DEGREES);
        }
        if(gamepad1.a){
            robot.arm.setTargetPosition(robot.ARM_LOWERED_VAL);
//            robot.arm.setPower(Math.max((Math.abs(robot.arm.getCurrentPosition() - robot.arm.getTargetPosition())) / 100 * (0.2), (0.2)));
            robot.arm.setVelocity(300, AngleUnit.DEGREES);
        }
        if(gamepad1.b){
            robot.arm.setTargetPosition(robot.ARM_LEVEL_VAL);
//            robot.arm.setPower(Math.max((Math.abs(robot.arm.getCurrentPosition() - robot.arm.getTargetPosition())) / 100 * (0.2), (0.2)));
            robot.arm.setVelocity(300, AngleUnit.DEGREES);

            robot.cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.cascade.setTargetPosition(robot.CASCADE_SCORE_DEFAULT_VAL);
            robot.cascade.setPower(0.9);

            //experimental
            intakeSlowMode = true;
            //end experimental
        }
        if(gamepad1.left_stick_button){
            robot.cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.cascade.setTargetPosition(robot.CASCADE_SCORE_FAR_VAL);
            robot.cascade.setPower(0.9);
        }
        if(gamepad1.y){
            robot.arm.setTargetPosition(robot.ARM_VERTICAL_VAL);
//            robot.arm.setPower(Math.max((Math.abs(robot.arm.getCurrentPosition() - robot.arm.getTargetPosition())) / 100 * (0.2), (0.2)));
            robot.arm.setVelocity(300, AngleUnit.DEGREES);
        }
        if(gamepad1.x){
//
//            //for easy pickup
////            robot.arm.setTargetPosition(robot.ARM_PICK_VAL);
////            robot.arm.setVelocity(300, AngleUnit.DEGREES);
////
////            robot.cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            robot.cascade.setTargetPosition(robot.CASCADE_SCORE_FAR_VAL);
////            robot.cascade.setPower(0.9);
//////            robot.arm.setTargetPosition(robot.ARM_BACK_VAL);
//////            robot.arm.setPower(Math.max((Math.abs(robot.arm.getCurrentPosition() - robot.arm.getTargetPosition())) / 100 * (0.2), (0.2)));
//
//            //for angled deposit
//            robot.arm.setTargetPosition(robot.ARM_VERTICAL_PLUS_VAL);
////            robot.arm.setPower(Math.max((Math.abs(robot.arm.getCurrentPosition() - robot.arm.getTargetPosition())) / 100 * (0.2), (0.2)));
//            robot.arm.setVelocity(300, AngleUnit.DEGREES);
            robot.arm.setTargetPosition(robot.ARM_VERTICAL_VAL);
            robot.arm.setPower(Math.max((Math.abs(robot.arm.getCurrentPosition() - robot.arm.getTargetPosition())) / 100 * (0.2), (0.2)));
//            robot.arm.setVelocity(300, AngleUnit.DEGREES);

        }

        if(gamepad1.left_bumper){
            robot.arm.setTargetPosition(robot.arm.getTargetPosition()+50); //ARM INCREMENT UP
//            robot.arm.setPower(Math.max((Math.abs(robot.arm.getCurrentPosition() - robot.arm.getTargetPosition())) / 100 * (0.2), (0.8)));
            robot.arm.setVelocity(150, AngleUnit.DEGREES);
        }
        if(gamepad1.right_bumper){
            robot.arm.setTargetPosition(robot.arm.getTargetPosition()-50); //ARM INCREMENT DOWN
//            robot.arm.setPower(Math.max((Math.abs(robot.arm.getCurrentPosition() - robot.arm.getTargetPosition())) / 100 * (0.2), (0.8)));
            robot.arm.setVelocity(150, AngleUnit.DEGREES);
        }




//        else{
////            robot.arm.setPower(expo(armPower));
//        }
        //END ARM


        //START CASCADE
        double cascadePower = 0;
        if(Math.abs(gamepad1.left_stick_y)>0.05) {
            cascadePower = gamepad1.left_stick_y;
            robot.cascade.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.cascade.setPower(cascadePower);

            intakeSlowMode = false;

        }else if(robot.cascade.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)){
            robot.cascade.setPower(cascadePower);
        }
        if(robot.cascade.getCurrentPosition() <= robot.CASCADE_MAX_VAL && cascadePower < 0) robot.cascade.setPower(0);
        if(robot.cascade.getCurrentPosition() >= robot.CASCADE_IN_VAL && cascadePower > 0) robot.cascade.setPower(0);

        //END CASCADE


        //START LIFT
        double liftPower = -(gamepad1.right_trigger-gamepad1.left_trigger);
        if(!robot.lift.isBusy()){
            robot.lift.setPower(0);
        }
        if(Math.abs(liftPower)>=0.1){
            if(!robot.lift.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER)){
                robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            robot.lift.setPower(liftPower);
        }else if(gamepad1.dpad_up){
            if(!robot.lift.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)){
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            robot.lift.setTargetPosition(robot.LIFT_HOOK_VAL);
            robot.lift.setPower(-1);
        }else if(gamepad1.dpad_down){
            if(!robot.lift.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)){
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            robot.lift.setTargetPosition(robot.LIFT_FULL_LIFT_VAL);
            robot.lift.setPower(1);
        }else if(gamepad1.dpad_left){ //LINING UP FOR HOOK
            if(!robot.lift.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)){
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            robot.lift.setTargetPosition(robot.LIFT_SMALL_LIFT_VAL);
            robot.lift.setPower(1);
        }else if(gamepad1.dpad_right){ //RESET POSITION FOR SCORING
            if(!robot.lift.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)){
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            robot.lift.setTargetPosition(robot.LIFT_DETATCH_VAL);
            robot.lift.setPower(1);
        }
//        robot.lift.setPower(-Math.max(-1.0, Math.min((lift_motor_position-robot.lift.getCurrentPosition())/150, 1.0)));
        //END LIFT


        //START DEBUGGING CODE
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
        //END DEBUGGING CODE

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
