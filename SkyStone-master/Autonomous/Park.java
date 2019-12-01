package org.firstinspires.ftc.teamcode.treamcodde.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.treamcodde.HouseFly;

@Autonomous(name = "ParkOnly", group = "")
public class Park extends LinearOpMode {


    HouseFly robot = new HouseFly(hardwareMap);


    @Override
    public void runOpMode() {
        robot.setMotorPowers(0.5,0.5,0.5,0.5);
        sleep(100);
        robot.setMotorPowers(0,0,0,0);
        sleep(200);
        requestOpModeStop();
    }
}
