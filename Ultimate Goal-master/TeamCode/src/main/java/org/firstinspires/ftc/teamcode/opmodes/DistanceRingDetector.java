package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp
public class DistanceRingDetector extends TunableLinearOpMode {
    private Rev2mDistanceSensor sensor;

    @Override
    public void runOpMode() {
        sensor = hardwareMap.get(Rev2mDistanceSensor.class, "2m REV Sensor");

        telemetry.addLine("press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            // have to add Locale.US since android lint gives a warning its cringe
            telemetry.addData("range", String.format(Locale.US, "%01f mm", sensor.getDistance(DistanceUnit.MM)));
            telemetry.update();
        }
    }

}
