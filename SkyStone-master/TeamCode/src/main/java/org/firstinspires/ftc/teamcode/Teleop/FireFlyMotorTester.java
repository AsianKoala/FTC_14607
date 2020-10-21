package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;


@TeleOp(name = "FireFlyRobot Motor Tester")
public class FireFlyMotorTester extends TunableOpMode {

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

        robot.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        }



        if(testing_active) {
            robot.leftSlide.setPower(gamepad1.left_stick_y);
            robot.rightSlide.setPower(gamepad1.right_stick_y);
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