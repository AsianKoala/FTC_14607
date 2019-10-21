package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Reset Encoders", group = "Dragonfly")

public class DragonflyAutoResetEncoders extends LinearOpMode {
    HardwareDragonfly robot = new HardwareDragonfly();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        robot.resetEncoders();
        telemetry.addData("Say", "Hello Driver");
        updateTelemetry(telemetry);

        robot.lift.setPower(0); // brake lift motor to keep robot hanging in place

        /** Wait for the game to begin */
        telemetry.addData("status", "waiting for start");
        updateTelemetry(telemetry);
        waitForStart();

        while (opModeIsActive()){ // wait until end of opmode
            telemetry.addData("Autonomous completed... ", 0);
        }
    }

}