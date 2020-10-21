package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Teleop.FireFlyRobot;

import java.util.HashMap;


@Autonomous(name = "REAL Park Blue Auto", group = "Firefly")
public class ParkBlue extends LinearOpMode {

    FireFlyRobot robot = new FireFlyRobot();

    double pauseStart = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initPositions();
//
        while(!isStarted() && !isStopRequested()) {

            if(Math.abs(gamepad1.left_stick_y) > 0.1) {
                pauseStart += gamepad1.left_stick_y/10;

            }
            telemetry.addData("Waiting Time: ", pauseStart);
            telemetry.addData("Ready.", 0);
            telemetry.update();
        }

        int state = 1;

        double flpower = 0;
        double frpower = 0;
        double blpower = 0;
        double brpower = 0;

        long stateStartTime = System.currentTimeMillis();

        HashMap<String, Double> powers = new HashMap<String, Double>();
        powers.put("fl", 0.0);
        powers.put("fr", 0.0);
        powers.put("bl", 0.0);
        powers.put("br", 0.0);

        while(opModeIsActive()) {

            powers.put("fl", 0.0);
            powers.put("fr", 0.0);
            powers.put("bl", 0.0);
            powers.put("br", 0.0);

            if(state == 1) {
                if(System.currentTimeMillis() - stateStartTime < pauseStart) {

                } else {
                    state = 2;
                    stateStartTime = System.currentTimeMillis();
                }
            }

            if(state == 2) {
                if(System.currentTimeMillis() - stateStartTime < 750) {
                    powers.put("fl", -0.35);
                    powers.put("fr", -0.4);
                    powers.put("bl", -0.35);
                    powers.put("br", -0.4);
                } else {
                    state = 2;
                    stateStartTime = System.currentTimeMillis();
                }
            }


            robot.setFLPower(powers.get("fl"));
            robot.setFRPower(powers.get("fr"));
            robot.setBLPower(powers.get("bl"));
            robot.setBRPower(powers.get("br"));

            telemetry.addData("state: ", state);
            telemetry.addData("power fl: ", powers.get("fl"));
            telemetry.addData("power fr: ", powers.get("fr"));
            telemetry.addData("power bl: ", powers.get("bl"));
            telemetry.addData("power br: ", powers.get("br"));
            telemetry.update();
        }

        robot.setDriveStop();

    }



}