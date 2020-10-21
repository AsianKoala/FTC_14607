package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Teleop.FireFlyRobot;

import java.util.HashMap;


@Autonomous(name = "REAL Park Horizontal Auto", group = "Firefly")
public class ParkHorizontal extends LinearOpMode {

    FireFlyRobot robot = new FireFlyRobot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initPositions();
//
        while(!isStarted() && !isStopRequested()) {
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
                if(System.currentTimeMillis() - stateStartTime < 2000) {
                    robot.setHorizontalExtendOut();
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