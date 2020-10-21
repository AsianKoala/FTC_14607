package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;
import org.firstinspires.ftc.teamcode.HelperClasses.ppProject.company.Range;

import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.*;


@TeleOp(name = "FireFlyRobot Teleop")
public class FireFlyTeleop extends TunableOpMode {

    FireFlyRobot robot = new FireFlyRobot();

    @Override
    public void init() {

        robot.init(hardwareMap);

        robot.initPositions();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ((ServoImplEx) (robot.clawRotater)).setPwmDisable();
        ((ServoImplEx) (robot.clawFlipper)).setPwmDisable();
        ((ServoImplEx) (robot.clawGripper)).setPwmDisable();
        // TODO: PROGRAM CLAW IN TELEOP

        robot.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    public void loop() {
        /// TUNABLE STUFF
//        horizontalExtendHome = getDouble("horizontalExtendHome");
//        horizontalExtendOut = getDouble("horizontalExtendOut");
//        horizontalExtendFeed = getDouble("horizontalExtendFeed");
//        rightHookGrip = getDouble("rightHookGrip");
//        leftHookGrip = getDouble("leftHookGrip");
//        leftHookHome = getDouble("leftHookHome");
//        rightHookHome = getDouble("rightHookHome");
//        capstoneDeployerOut = getDouble("capstoneDeployerOut");
//        capstoneDeployerHome = getDouble("capstoneDeployerHome");
//        frontGripperOpen = getDouble("frontGripperOpen");
//        frontGripperClosed = getDouble("frontGripperClosed");
//        backGripperOpen = getDouble("backGripperOpen");
//        backGripperClosed = getDouble("backGripperClosed");
//        parkingDeployerOut = getDouble("parkingDeployerOut");
//        parkingDeployerIn = getDouble("parkingDeployerIn");








        // INTAKE CONTROLS
        double intakeMultiplier = 0.6;
        if(gamepad2.left_stick_button) {
            intakeMultiplier = 1.0;
        }
        double intakePower = gamepad2.left_stick_y;
        double leftIntakePower = intakePower;// - gamepad2.left_stick_x;
        double rightIntakePower = intakePower;// + gamepad2.left_stick_x;
        if(Math.abs(intakePower) < 0.1) {
            robot.setIntakeStop();
        }else {
            robot.setLeftIntakePower(intakeMultiplier * -leftIntakePower);
            robot.setRightIntakePower(intakeMultiplier * -rightIntakePower);
        }


        // DRIVETRAIN CONTROLS
        double motorPower = 1;
//        if(gamepad1.left_bumper) {
        if(gamepad1.right_trigger > 1.1) { // not possible
            motorPower = 0.5;
        }
//        else if(gamepad1.right_bumper) {
        else if(gamepad1.right_trigger > 0.1) { // 0.5
//            motorPower = 0.25;
            motorPower = Range.clip(1-gamepad1.right_trigger, 0.25, 1);
        }
        else if(gamepad1.right_bumper) {
            motorPower = 0.5;
        }
        double threshold = 0.157;
        if(Math.abs(gamepad1.left_stick_y) > threshold || Math.abs(gamepad1.left_stick_x) > threshold || Math.abs(gamepad1.right_stick_x) > threshold)
        {
            robot.setFRPower(motorPower * (((-gamepad1.left_stick_y) + (gamepad1.left_stick_x)) + -gamepad1.right_stick_x));
            robot.setBLPower(motorPower * (((-gamepad1.left_stick_y) + (-gamepad1.left_stick_x)) + gamepad1.right_stick_x));
            robot.setFLPower(motorPower * (((-gamepad1.left_stick_y) + (gamepad1.left_stick_x)) + gamepad1.right_stick_x));
            robot.setBRPower(motorPower * (((-gamepad1.left_stick_y) + (-gamepad1.left_stick_x)) + -gamepad1.right_stick_x));
        }
        else
        {
            robot.setDriveStop();
        }


        // LIFT CONTROLS
//        double liftPower = 0.8;
//        double slideMultiplier = 20;
//        if(gamepad2.right_stick_button) {
//            slideMultiplier = 80;
//            liftPower = 0.99;
//        }
//        double liftInputValue = gamepad2.right_stick_y;
//        int increment = (int) (liftInputValue * slideMultiplier);
//        if(Math.abs(liftInputValue) >= 0.1) {
////            robot.setLiftTargetPosition(robot.getLiftPosition()+increment, liftPower);
//            robot.setLiftTargetPosition(robot.getLiftTargetPosition()+increment, liftPower);
//        }
////        if(gamepad2.x) {
////            robot.setLiftTargetPositionHome(0.7);
////        }

        if(Math.abs(gamepad2.right_stick_y) > 0.3) {
            if(gamepad2.right_stick_y > 0) {
                if(robot.getLiftPosition() > -100) {
                    robot.leftSlide.setPower(gamepad2.right_stick_y);
                    robot.rightSlide.setPower(gamepad2.right_stick_y);
                }else{
                    if(gamepad2.right_stick_button){
                        robot.leftSlide.setPower(0.0001); // 0.1 0.025
                        robot.rightSlide.setPower(0.0001); // 0.1
                    }else {
                        robot.leftSlide.setPower(-0.05); // 0.1 0.025
                        robot.rightSlide.setPower(-0.05); // 0.1
                    }

                }

            }else{
                robot.leftSlide.setPower(gamepad2.right_stick_y);
                robot.rightSlide.setPower(gamepad2.right_stick_y);
            }

        }else if(robot.getLiftPosition() < -300) { // -300
            robot.leftSlide.setPower(-0.3);
            robot.rightSlide.setPower(-0.3);
        }else if(robot.getLiftPosition() < -100) { // -300
            robot.leftSlide.setPower(-0.2);
            robot.rightSlide.setPower(-0.2);
        }else {
            robot.leftSlide.setPower(0);
            robot.rightSlide.setPower(0);
        }






        // OUTTAKE CONTROLS
        if(gamepad2.b) {
            robot.setFrontGripperOpen();
            robot.setBackGripperOpen();
        }
        if(gamepad2.a) {
            robot.setFrontGripperClosed();
            robot.setBackGripperClosed();
        }
        if(gamepad2.x) {
            robot.setFrontGripperOpen();
            robot.setBackGripperClosed();
        }

        if(gamepad2.right_trigger > 0.4) {
            robot.setHorizontalExtendOut();
        }
        if(gamepad2.left_trigger > 0.4) {
            robot.setHorizontalExtendHome();
        }

        if(gamepad2.y) {
            robot.setFrontGripperClosed();
            robot.setBackGripperOpen();
            robot.setHorizontalExtendFeed();
        }

        if(gamepad2.dpad_down) {
            robot.setBackGripperClosed();
            robot.setCapstoneHome();
            robot.setFrontGripperClosed();
            robot.setHorizontalExtendHome();
        }

        if(gamepad1.dpad_down) {
            robot.setBackGripperCap();
            robot.setFrontGripperClosed();
            robot.setHorizontalExtendOut();
            robot.setCapstoneOut();
            robot.setFoundationHookHome();
        }

        if(gamepad1.dpad_up) {
            robot.setBackGripperClosed();
            robot.setCapstoneHome();
            robot.setFrontGripperClosed();
            robot.setHorizontalExtendHome();
            robot.setFoundationHookHome();
        }

        if(gamepad1.dpad_left) {
            robot.setFrontGripperClosed();
        }


        // CAPSTONE CONTROLS
        if(gamepad1.left_trigger > 0.25) {
//            robot.setCapstoneOut();
            robot.setCapstonePosition(robot.getCapstonePosition() - gamepad1.left_trigger/50);
            robot.setBackGripperCap();

//            robot.setLiftTargetPosition(robot.getLiftPosition(), 0.3);
//            robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(gamepad1.left_trigger > 0.95) {
//            robot.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setBackGripperClosed();
            robot.setCapstoneHome();
        }
        if(gamepad1.dpad_right) {
            robot.setHorizontalExtendManual(robot.getHorizontalExtendPosition()+0.05);
        }
        if(gamepad1.dpad_left) {
            robot.setHorizontalExtendManual(robot.getHorizontalExtendPosition()-0.05);
        }
//        if(gamepad1.dpad_down) {
//            robot.setLiftTargetPosition(robot.getLiftPosition()+1, 0.1);
//        }
//        if(gamepad1.dpad_up) {
//            robot.setLiftTargetPosition(robot.getLiftPosition()-1,    0.1);
//        }


        // SIDE GRIPPER ARM CONTROLS
//TODO


        // PARKING MECH CONTROLS
        if(gamepad1.y) {
            robot.setParkingDeployerPosition(robot.getParkingDeployerPosition()+0.02);
        }
        if(gamepad1.x) {
            robot.setParkingDeployerPosition(robot.getParkingDeployerPosition()-0.02);
        }


        // FOUNDATION MOVER CONTROLS
//        if(!gamepad1.right_bumper && !gamepad1.a) {
        if(!(gamepad1.right_trigger>0.5 || gamepad1.right_bumper) && !gamepad1.a) {
            robot.setFoundationHookHome();
        }
        if(gamepad1.b) {
            robot.setFoundationHookHome();
        }
        if(gamepad1.a) {
            robot.setFoundationHookGrip();
        }







//        telemetry.addData("horizontal extend home", horizontalExtendHome);
//        telemetry.addData("horizontal extend out", horizontalExtendOut);
//        telemetry.addData("horizontal extend feed", horizontalExtendFeed);
//        telemetry.addData("right hook grip", rightHookGrip);
//        telemetry.addData("left hook grip", leftHookGrip);
//        telemetry.addData("right hook home", rightHookHome);
//        telemetry.addData("left hook home", leftHookHome);
//        telemetry.addData("capstone deployer out", capstoneDeployerOut);
//        telemetry.addData("capstone deployer home", capstoneDeployerHome);
//        telemetry.addData("front gripper open", frontGripperOpen);
//        telemetry.addData("front gripper closed", frontGripperClosed);
//        telemetry.addData("back gripper open", backGripperOpen);
//        telemetry.addData("back gripper closed", backGripperClosed);
//        telemetry.addData("parking deployer in", parkingDeployerIn);
//        telemetry.addData("parking deployer out", parkingDeployerOut);
        telemetry.addData("target lift val: ", robot.liftTargetPosition);
//        telemetry.addData("increment: ", increment);
//        telemetry.addData("lift power: ", liftPower);
        telemetry.addData("odo pod 1: (forward back) ", robot.leftIntake.getCurrentPosition());
        telemetry.addData("odo pod 2: (left right) ", robot.rightIntake.getCurrentPosition());
        telemetry.addData("leftslide: ", robot.leftSlide.getCurrentPosition());
        telemetry.addData("rightslide: ", robot.rightSlide.getCurrentPosition());
        telemetry.update();
    }
}