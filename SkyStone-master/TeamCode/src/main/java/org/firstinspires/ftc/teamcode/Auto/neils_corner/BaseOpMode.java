package org.firstinspires.ftc.teamcode.Auto.neils_corner;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Auto.roadrunner.util.BNO055IMUUtil;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.ArrayList;


public class BaseOpMode extends LinearOpMode {

    protected ExpansionHubMotor leftFront, leftRear, rightFront, rightRear;
    protected ExpansionHubMotor horizontalModule, verticalModule;
    protected ExpansionHubMotor leftSlide, rightSlide;


    protected ArrayList<ExpansionHubMotor> driveMotors = new ArrayList<ExpansionHubMotor>() {{
        add(leftFront);
        add(leftRear);
        add(rightFront);
        add(rightRear);
    }};


    protected BNO055IMU imu;

    protected ExpansionHubEx masterHub, slaveHub;


    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(ExpansionHubMotor.class, "leftFront");
        leftRear = hardwareMap.get(ExpansionHubMotor.class, "leftRear");
        rightFront = hardwareMap.get(ExpansionHubMotor.class, "rightFront");
        rightRear = hardwareMap.get(ExpansionHubMotor.class, "rightRear");
        horizontalModule = hardwareMap.get(ExpansionHubMotor.class, "horizontalModule");
        verticalModule = hardwareMap.get(ExpansionHubMotor.class, "verticalModule");
        leftSlide = hardwareMap.get(ExpansionHubMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(ExpansionHubMotor.class, "rightSlide");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        masterHub = hardwareMap.get(ExpansionHubEx.class, "masterHub");
        slaveHub = hardwareMap.get(ExpansionHubEx.class, "slaveHub");


        for(ExpansionHubMotor motor : driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        horizontalModule.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalModule.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);


        imu.initialize(new BNO055IMU.Parameters());
        // TODO: BNO055IMUUtil.remapAxes(imu, something something);



    }

}