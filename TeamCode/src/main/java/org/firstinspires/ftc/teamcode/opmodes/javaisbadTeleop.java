package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.ArrayList;

@TeleOp
public class javaisbadTeleop extends OpMode {

    private drivetrain driveTrain;

    public void init() {
        ExpansionHubMotor frontLeft = hardwareMap.get(ExpansionHubMotor.class, "FL");
        ExpansionHubMotor frontRight = hardwareMap.get(ExpansionHubMotor.class, "FR");
        ExpansionHubMotor backLeft = hardwareMap.get(ExpansionHubMotor.class, "BL");
        ExpansionHubMotor backRight = hardwareMap.get(ExpansionHubMotor.class, "BR");
        driveTrain = new drivetrain(frontLeft, frontRight, backLeft, backRight);
    }

    public void loop() {
        double scale = gamepad1.left_bumper ? 1 : 0.5;
        driveTrain.powers = new Pose(
                -gamepad1.left_stick_x * scale,
                gamepad1.left_stick_y * scale,
                -gamepad1.right_stick_x *scale
        );
        driveTrain.update();
        telemetry.addData("left", driveTrain.frontLeft.getCurrentPosition());
        telemetry.addData("right", driveTrain.frontRight.getCurrentPosition());
        telemetry.addData("perp", driveTrain.backLeft.getCurrentPosition());
    }
}

class Pose {
    public double x,y,h;
    public Pose(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }
}

class drivetrain {
    ExpansionHubMotor frontLeft, frontRight, backLeft, backRight;
    public Pose powers;
    private final ArrayList<ExpansionHubMotor> motors = new ArrayList<>();

    public drivetrain(ExpansionHubMotor fl, ExpansionHubMotor fr,
                      ExpansionHubMotor bl, ExpansionHubMotor br) {
        frontLeft = fl;
        frontRight = fr;
        backLeft = bl;
        backRight = br;
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motors.add(frontLeft);
        motors.add(frontRight);
        motors.add(backLeft);
        motors.add(backRight);
        for(ExpansionHubMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        powers = new Pose(0,0,0);
    }
    public void update() {
        double rawFL = powers.y + powers.x + powers.h;
        double rawFR = powers.y - powers.x - powers.h;
        double rawBL = powers.y - powers.x + powers.h;
        double rawBR = powers.y + powers.x - powers.h;
        double[] rawPowers = {rawFL, rawFR, rawBL, rawBR};
        double maxabs = rawPowers[0];
        for(double i : rawPowers) {
            if(Math.abs(i) > maxabs) {
                maxabs = Math.abs(i);
            }
        }
        if(maxabs > 1) {
            for(int i=0; i<rawPowers.length; i++) {
                rawPowers[i] /= maxabs;
            }
        }
        for(int i=0; i<rawPowers.length; i++) {
            motors.get(i).setPower(rawPowers[i]);
        }
    }
}
