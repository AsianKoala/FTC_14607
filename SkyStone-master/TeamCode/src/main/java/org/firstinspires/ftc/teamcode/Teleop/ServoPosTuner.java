package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;
import org.firstinspires.ftc.teamcode.HelperClasses.ppProject.company.Range;
import org.openftc.revextensions2.ExpansionHubServo;

import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.*;

@TeleOp
public class ServoPosTuner extends TunableOpMode {

    private Servo leftHook, rightHook, capstoneDeployer, parkingDeployer, horizontalExtend, frontGripper, backGripper;



    @Override
    public void init() {
        leftHook = hardwareMap.get(Servo.class, "leftHook");
        rightHook = hardwareMap.get(Servo.class, "rightHook");
        capstoneDeployer = hardwareMap.get(Servo.class, "capstoneDeployer");
        parkingDeployer = hardwareMap.get(Servo.class, "parkingDeployer");
        horizontalExtend = hardwareMap.get(Servo.class, "horizontalExtend");
        frontGripper = hardwareMap.get(Servo.class, "frontGripper");
        backGripper = hardwareMap.get(Servo.class, "backGripper");

    }

    @Override
    public void loop() {

        horizontalExtendHome = getDouble("horizontalExtendHome");
        horizontalExtendOut = getDouble("horizontalExtendOut");
        horizontalExtendFeed = getDouble("horizontalExtendFeed");
        rightHookGrip = getDouble("rightHookGrip");
        leftHookGrip = getDouble("leftHookGrip");
        leftHookHome = getDouble("leftHookHome");
        rightHookHome = getDouble("rightHookHome");
        capstoneDeployerOut = getDouble("capstoneDeployerOut");
        capstoneDeployerHome = getDouble("capstoneDeployerHome");
        frontGripperOpen = getDouble("frontGripperOpen");
        frontGripperClosed = getDouble("frontGripperClosed");
        backGripperOpen = getDouble("backGripperOpen");
        backGripperClosed = getDouble("backGripperClosed");
        parkingDeployerOut = getDouble("parkingDeployerOut");
        parkingDeployerIn = getDouble("parkingDeployerIn");


        if(gamepad1.a) {
            horizontalExtend.setPosition(horizontalExtendHome);
        }

        if(gamepad1.b) {
            horizontalExtend.setPosition(horizontalExtendFeed);
        }

        if(gamepad1.y) {
            horizontalExtend.setPosition(horizontalExtendOut);
        }

        if(gamepad1.dpad_up) {
            leftHook.setPosition(leftHookGrip);
        }

        if(gamepad1.dpad_down) {
            leftHook.setPosition(leftHookHome);
        }

        if(gamepad1.dpad_left) {
            rightHook.setPosition(rightHookGrip);
        }

        if(gamepad1.dpad_right) {
            rightHook.setPosition(rightHookHome);
        }

        if(gamepad1.left_bumper) {
            capstoneDeployer.setPosition(capstoneDeployerHome);
        }

        if(gamepad1.left_trigger > 0.5) {
            capstoneDeployer.setPosition(capstoneDeployerOut);
        }

        if(gamepad1.right_bumper) {
            parkingDeployer.setPosition(parkingDeployerIn);
        }

        if(gamepad1.right_trigger > 0.5) {
            parkingDeployer.setPosition(parkingDeployerOut);
        }

        if(gamepad2.left_bumper) {
            frontGripper.setPosition(frontGripperOpen);
        }

        if(gamepad2.left_trigger > 0.5) {
            frontGripper.setPosition(frontGripperClosed);
        }

        if(gamepad2.right_bumper) {
            backGripper.setPosition(backGripperOpen);
        }

        if(gamepad2.right_trigger > 0.5) {
            backGripper.setPosition(backGripperClosed);
        }





        telemetry.addData("left hook", leftHook.getPosition());
        telemetry.addData("right hook", rightHook.getPosition());
        telemetry.addData("capstone deployer", capstoneDeployer.getPosition());
        telemetry.addData("parking deployer", parkingDeployer.getPosition());
        telemetry.addData("horizontal extend", horizontalExtend.getPosition());
        telemetry.addData("front gripper", frontGripper.getPosition());
        telemetry.addData("back gripper", backGripper.getPosition());
        for(int i=0; i<5; i++) telemetry.addLine();
        telemetry.addData("horizontal extend home", horizontalExtendHome);
        telemetry.addData("horizontal extend out", horizontalExtendOut);
        telemetry.addData("horizontal extend feed", horizontalExtendFeed);
        telemetry.addData("right hook grip", rightHookGrip);
        telemetry.addData("left hook grip", leftHookGrip);
        telemetry.addData("right hook home", rightHookHome);
        telemetry.addData("left hook home", leftHookHome);
        telemetry.addData("capstone deployer out", capstoneDeployerOut);
        telemetry.addData("capstone deployer home", capstoneDeployerHome);
        telemetry.addData("front gripper open", frontGripperOpen);
        telemetry.addData("front gripper closed", frontGripperClosed);
        telemetry.addData("back gripper open", backGripperOpen);
        telemetry.addData("back gripper closed", backGripperClosed);
        telemetry.addData("parking deployer in", parkingDeployerIn);
        telemetry.addData("parking deployer out", parkingDeployerOut);
        telemetry.update();
    }
}
