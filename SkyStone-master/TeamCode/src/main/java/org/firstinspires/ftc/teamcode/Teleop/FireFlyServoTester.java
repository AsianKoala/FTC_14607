package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.*;


@TeleOp(name = "FireFlyRobot Servo Tester")
public class FireFlyServoTester extends TunableOpMode {

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
            telemetry.addData("testing servo clawRotater", "");
            telemetry.update();
        }
        if(gamepad1.dpad_down) {
            ((ServoImplEx) (robot.clawFlipper)).setPwmEnable();
            current_testing_servo = 2;
            telemetry.addData("testing servo clawFlipper", "");
            telemetry.update();
        }
        if(gamepad1.dpad_left) {
            ((ServoImplEx) (robot.clawGripper)).setPwmEnable();
            current_testing_servo = 3;
            telemetry.addData("testing servo clawGripper", "");
            telemetry.update();
        }
        if(gamepad1.dpad_right) {
            ((ServoImplEx) (robot.frontGripper)).setPwmEnable();
            current_testing_servo = 4;
            telemetry.addData("testing servo frontGripper", "");
            telemetry.update();
        }
        if(gamepad1.left_bumper) {
            ((ServoImplEx) (robot.backGripper)).setPwmEnable();
            current_testing_servo = 5;
            telemetry.addData("testing servo backGripper", "");
            telemetry.update();
        }
        if(gamepad1.right_bumper) {
            ((ServoImplEx) (robot.horizontalExtend)).setPwmEnable();
            current_testing_servo = 6;
            telemetry.addData("testing servo horizontalExtend", "");
            telemetry.update();
        }
        if(gamepad1.left_stick_button) {
            ((ServoImplEx) (robot.leftHook)).setPwmEnable();
            current_testing_servo = 7;
            telemetry.addData("testing servo leftHook", "");
            telemetry.update();
        }
        if(gamepad1.right_stick_button) {
            ((ServoImplEx) (robot.rightHook)).setPwmEnable();
            current_testing_servo = 8;
            telemetry.addData("testing servo rightHook", "");
            telemetry.update();
        }
        if(gamepad1.x) {
            ((ServoImplEx) (robot.parkingDeployer)).setPwmEnable();
            current_testing_servo = 9;
            telemetry.addData("testing servo parkingDeployer", "");
            telemetry.update();
        }
        if(gamepad1.y) {
            ((ServoImplEx) (robot.capstoneDeployer)).setPwmEnable();
            current_testing_servo = 10;
            telemetry.addData("testing servo capstoneDeployer", "");
            telemetry.update();
        }

        if(testing_active) {
            if(current_testing_servo == 1) {
                if(System.currentTimeMillis()/1000 % 2 == 0) {
                    robot.clawRotater.setPosition(0.1);
                }
                if(System.currentTimeMillis()/1000 % 2 == 1) {
                    robot.clawRotater.setPosition(0.9);
                }
            } else if(current_testing_servo == 2) {
                if(System.currentTimeMillis()/1000 % 2 == 0) {
                    robot.clawFlipper.setPosition(0.1);
                }
                if(System.currentTimeMillis()/1000 % 2 == 1) {
                    robot.clawFlipper.setPosition(0.9);
                }
            } else if(current_testing_servo == 3) {
                if(System.currentTimeMillis()/1000 % 2 == 0) {
                    robot.clawGripper.setPosition(0.1);
                }
                if(System.currentTimeMillis()/1000 % 2 == 1) {
                    robot.clawGripper.setPosition(0.9);
                }
            } else if(current_testing_servo == 4) {
                if(System.currentTimeMillis()/1000 % 2 == 0) {
                    robot.frontGripper.setPosition(0.1);
                }
                if(System.currentTimeMillis()/1000 % 2 == 1) {
                    robot.frontGripper.setPosition(0.9);
                }
            } else if(current_testing_servo == 5) {
                if(System.currentTimeMillis()/1000 % 2 == 0) {
                    robot.backGripper.setPosition(0.1);
                }
                if(System.currentTimeMillis()/1000 % 2 == 1) {
                    robot.backGripper.setPosition(0.9);
                }
            } else if(current_testing_servo == 6) {
                if(System.currentTimeMillis()/1000 % 2 == 0) {
                    robot.horizontalExtend.setPosition(0.1);
                }
                if(System.currentTimeMillis()/1000 % 2 == 1) {
                    robot.horizontalExtend.setPosition(0.9);
                }
            } else if(current_testing_servo == 7) {
                if(System.currentTimeMillis()/1000 % 2 == 0) {
                    robot.leftHook.setPosition(0.1);
                }
                if(System.currentTimeMillis()/1000 % 2 == 1) {
                    robot.leftHook.setPosition(0.9);
                }
            } else if(current_testing_servo == 8) {
                if(System.currentTimeMillis()/1000 % 2 == 0) {
                    robot.rightHook.setPosition(0.1);
                }
                if(System.currentTimeMillis()/1000 % 2 == 1) {
                    robot.rightHook.setPosition(0.9);
                }
            } else if(current_testing_servo == 9) {
                if(System.currentTimeMillis()/1000 % 2 == 0) {
                    robot.parkingDeployer.setPosition(0.1);
                }
                if(System.currentTimeMillis()/1000 % 2 == 1) {
                    robot.parkingDeployer.setPosition(0.9);
                }
            } else if(current_testing_servo == 10) {
                if(System.currentTimeMillis()/1000 % 2 == 0) {
                    robot.capstoneDeployer.setPosition(0.1);
                }
                if(System.currentTimeMillis()/1000 % 2 == 1) {
                    robot.capstoneDeployer.setPosition(0.9);
                }
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