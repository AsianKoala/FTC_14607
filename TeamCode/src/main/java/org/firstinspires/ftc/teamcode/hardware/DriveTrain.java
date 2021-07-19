package org.firstinspires.ftc.teamcode.hardware;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Pose;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.Arrays;
import java.util.SortedMap;
import java.util.TreeMap;

public class DriveTrain extends Hardware {

    public static Pose powers = new Pose();
    private final ExpansionHubMotor[] motors;
    public DriveTrain(ExpansionHubMotor frontLeft, ExpansionHubMotor frontRight, ExpansionHubMotor backLeft, ExpansionHubMotor backRight) {
        motors = new ExpansionHubMotor[]{frontLeft, frontRight, backLeft, backRight};
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        for(ExpansionHubMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        powers = new Pose();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public SortedMap<String, Object> update() {
        double rawFrontLeft = powers.y + powers.x + powers.heading;
        double rawFrontRight =  powers.y - powers.x - powers.heading;
        double rawBackLeft = powers.y - powers.x + powers.heading;
        double rawBackRight =  powers.y + powers.x - powers.heading;
        double[] rawPowers = {rawFrontLeft, rawFrontRight, rawBackLeft, rawBackRight};

        double maxAbsPower = Math.abs(rawFrontLeft);
        for(double power : rawPowers) {
            if(Math.abs(power) > maxAbsPower)
                maxAbsPower = Math.abs(power);
        }

        if(maxAbsPower > 1) {
            for (int i = 0; i < rawPowers.length; i++)
                rawPowers[i] /= maxAbsPower;
        }
        for(int i=0; i<rawPowers.length; i++) {
            motors[i].setPower(rawPowers[i]);
        }

        SortedMap<String, Object> telemetryPacket = new TreeMap<>();
        telemetryPacket.put("power vectors", powers.toString());
        telemetryPacket.put("powers", Arrays.toString(rawPowers));
        return telemetryPacket;
    }
}
