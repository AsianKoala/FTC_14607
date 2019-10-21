package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.HouseFly_Hardware;

//test
@TeleOp(name="lastYearKindaScuffedIguessIdK", group="Dragonfly")
public class teleopscuffedaf extends OpMode{

    HouseFly_Hardware robot = new HouseFly_Hardware();
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
//        robot.resetEncoders(); TODO: RESET ARM ENCODERS DEPENDING ON AUTO RUN
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

        //END TELEMETRY


        //START DRIVE CODE
        double leftDrivePower = expo(gamepad2.left_stick_y);
        double rightDrivePower = expo(gamepad2.right_stick_y);
        if(Math.abs(leftDrivePower) < 0.05) leftDrivePower = 0;
        if(Math.abs(rightDrivePower) < 0.05) rightDrivePower = 0;
        if(gamepad2.right_trigger > 0.05){
            leftDrivePower = gamepad2.right_trigger/1.5;
            rightDrivePower = gamepad2.right_trigger/1.5;
        }
        if(gamepad2.left_trigger > 0.05){
            leftDrivePower = -gamepad2.left_trigger/1.5;
            rightDrivePower = -gamepad2.left_trigger/1.5;
        }

        if(gamepad2.dpad_up){
            leftDrivePower = -0.2;
            rightDrivePower = -0.2;
        }
        if(gamepad2.dpad_down){
            leftDrivePower = 0.2;
            rightDrivePower = 0.2;
        }
        if(gamepad2.dpad_right){
            leftDrivePower = -0.3;
            rightDrivePower = 0.3;
        }
        if(gamepad2.dpad_left){
            leftDrivePower = 0.3;
            rightDrivePower = -0.3;
        }

        if(gamepad2.left_bumper){
            leftDrivePower/=1.25;
            rightDrivePower/=1.25;
        }
        if(gamepad2.right_bumper){
            leftDrivePower = leftDrivePower*1.5;
            rightDrivePower = rightDrivePower*1.5;
        }
        //END DRIVE CODE


        //START ADDITIONAL TELEMETRY
        updateTelemetry(telemetry);


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
