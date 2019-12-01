package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.HelperClasses.HouseFly;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        HouseFly drive = new HouseFly(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.turnSync(Math.toRadians(ANGLE));
    }
}
