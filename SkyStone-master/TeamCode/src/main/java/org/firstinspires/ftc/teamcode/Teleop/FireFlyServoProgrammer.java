package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;
import org.firstinspires.ftc.teamcode.HelperClasses.ppProject.company.Range;


@TeleOp(name = "FireFlyRobot Servo Programmer")
public class FireFlyServoProgrammer extends TunableOpMode {

    FireFlyRobot robot = new FireFlyRobot();

    @Override
    public void init() {

        robot.init(hardwareMap);

        robot.initPositions();

        ((ServoImplEx) (robot.clawRotater)).setPwmDisable();
        ((ServoImplEx) (robot.clawFlipper)).setPwmDisable();
        ((ServoImplEx) (robot.clawGripper)).setPwmDisable();
        ((ServoImplEx) (robot.frontGripper)).setPwmDisable();
        ((ServoImplEx) (robot.backGripper)).setPwmDisable();
        ((ServoImplEx) (robot.horizontalExtend)).setPwmDisable();
        ((ServoImplEx) (robot.leftHook)).setPwmDisable();
        ((ServoImplEx) (robot.rightHook)).setPwmDisable();
        ((ServoImplEx) (robot.parkingDeployer)).setPwmDisable();
        ((ServoImplEx) (robot.capstoneDeployer)).setPwmDisable();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    boolean testing_active = false;
    int current_testing_servo = 0;
    double value = 0.5;

    public void loop() {

        if(gamepad1.a) {
            testing_active = true;
        }

        if(gamepad1.b) {
            testing_active = false;
            ((ServoImplEx) (robot.clawRotater)).setPwmDisable();
            ((ServoImplEx) (robot.clawFlipper)).setPwmDisable();
            ((ServoImplEx) (robot.clawGripper)).setPwmDisable();
            ((ServoImplEx) (robot.frontGripper)).setPwmDisable();
            ((ServoImplEx) (robot.backGripper)).setPwmDisable();
            ((ServoImplEx) (robot.horizontalExtend)).setPwmDisable();
            ((ServoImplEx) (robot.leftHook)).setPwmDisable();
            ((ServoImplEx) (robot.rightHook)).setPwmDisable();
            ((ServoImplEx) (robot.parkingDeployer)).setPwmDisable();
            ((ServoImplEx) (robot.capstoneDeployer)).setPwmDisable();
        }

        if(gamepad1.dpad_up) {
            ((ServoImplEx) (robot.clawRotater)).setPwmEnable();
            current_testing_servo = 1;
            telemetry.addData("test ", "testing servo clawRotater");
        }
        if(gamepad1.dpad_down) {
            ((ServoImplEx) (robot.clawFlipper)).setPwmEnable();
            current_testing_servo = 2;
            telemetry.addData("test ", "testing servo clawFlipper");
        }
        if(gamepad1.dpad_left) {
            ((ServoImplEx) (robot.clawGripper)).setPwmEnable();
            current_testing_servo = 3;
            telemetry.addData("test ", "testing servo clawGripper");
        }
        if(gamepad1.dpad_right) {
            ((ServoImplEx) (robot.frontGripper)).setPwmEnable();
            current_testing_servo = 4;
            telemetry.addData("test ", "testing servo frontGripper");
        }
        if(gamepad1.left_bumper) {
            ((ServoImplEx) (robot.backGripper)).setPwmEnable();
            current_testing_servo = 5;
            telemetry.addData("test ", "testing servo backGripper");
        }
        if(gamepad1.right_bumper) {
            ((ServoImplEx) (robot.horizontalExtend)).setPwmEnable();
            current_testing_servo = 6;
            telemetry.addData("test ", "testing servo horizontalExtend");
        }
        if(gamepad1.left_stick_button) {
            ((ServoImplEx) (robot.leftHook)).setPwmEnable();
            current_testing_servo = 7;
            telemetry.addData("test ", "testing servo leftHook");
        }
        if(gamepad1.right_stick_button) {
            ((ServoImplEx) (robot.rightHook)).setPwmEnable();
            current_testing_servo = 8;
            telemetry.addData("test ", "testing servo rightHook");
        }
        if(gamepad1.x) {
            ((ServoImplEx) (robot.parkingDeployer)).setPwmEnable();
            current_testing_servo = 9;
            telemetry.addData("test ", "testing servo parkingDeployer");
        }
        if(gamepad1.y) {
            ((ServoImplEx) (robot.capstoneDeployer)).setPwmEnable();
            current_testing_servo = 10;
            telemetry.addData("test ", "testing servo capstoneDeployer");
        }

        value += gamepad1.left_stick_y/500;
        value = Range.clip(value, 0, 1);
        telemetry.addData("value", value);
        telemetry.update();

        if(testing_active) {
            if(current_testing_servo == 1) {
                robot.clawRotater.setPosition(value);
            } else if(current_testing_servo == 2) {
                robot.clawFlipper.setPosition(value);
            } else if(current_testing_servo == 3) {
                robot.clawGripper.setPosition(value);
            } else if(current_testing_servo == 4) {
                robot.frontGripper.setPosition(value);
            } else if(current_testing_servo == 5) {
                robot.backGripper.setPosition(value);
            } else if(current_testing_servo == 6) {
                robot.horizontalExtend.setPosition(value);
            } else if(current_testing_servo == 7) {
                robot.leftHook.setPosition(value);
            } else if(current_testing_servo == 8) {
                robot.rightHook.setPosition(value);
            } else if(current_testing_servo == 9) {
                robot.parkingDeployer.setPosition(value);
            } else if(current_testing_servo == 10) {
                robot.capstoneDeployer.setPosition(value);
            }
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
    }
}