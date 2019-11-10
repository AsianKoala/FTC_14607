package org.firstinspires.ftc.teamcode.roadrunner.teamcode.drive.Hardware;

import android.os.SystemClock;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.revextensions2.ExpansionHubMotor;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.util.LynxOptimizedI2cFactory;
import org.openftc.revextensions2.ExpansionHubEx;

import static org.firstinspires.ftc.teamcode.roadrunner.teamcode.drive.Hardware.MovementVars.*;

import java.util.ArrayList;

@Config
public class DriveTrain extends SampleMecanumDriveBase {
    private ExpansionHubEx master;
    private ExpansionHubEx slave;
    public ExpansionHubMotor frontLeft;
    public ExpansionHubMotor frontRight;
    public ExpansionHubMotor backLeft;
    public ExpansionHubMotor backRight;
    public BNO055IMU imu;

    ArrayList<ExpansionHubMotor> leftMotors = new ArrayList<>();
    ArrayList<ExpansionHubMotor> rightMotors = new ArrayList<>();


    public DriveTrain(HardwareMap hwMap) {
        super();

        leftMotors.add(frontLeft);
        rightMotors.add(frontRight);
        leftMotors.add(backLeft);
        leftMotors.add(backRight);


        /**
         * we need to do 2 bulk reads since the drive motors are split across 2 hubs
         */

        slave = hwMap.get(ExpansionHubEx.class, "follower");

        for(ExpansionHubMotor expansionHubMotor : leftMotors) {
            expansionHubMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            expansionHubMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }


        /**
         * 2nd bulk read
         */

        master = hwMap.get(ExpansionHubEx.class, "master");

        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(master.getStandardModule(), 0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.PNP);


        for(ExpansionHubMotor expansionHubMotor : rightMotors) {
            expansionHubMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }




    }

    private long lastUpdateTime = 0;



    public void ApplyMovement() {
        // makes sure we dont spam
        long currTime = SystemClock.uptimeMillis();
        if(currTime - lastUpdateTime < 16) {
            return;
        }

        double tl_power_raw = movement_y-movement_turn+movement_x*1.5;
        double bl_power_raw = movement_y-movement_turn- movement_x*1.5;
        double br_power_raw = -movement_y-movement_turn-movement_x*1.5;
        double tr_power_raw = -movement_y-movement_turn+movement_x*1.5;





        //find the maximum of the powers
        double maxRawPower = Math.abs(tl_power_raw);
        if(Math.abs(bl_power_raw) > maxRawPower){ maxRawPower = Math.abs(bl_power_raw);}
        if(Math.abs(br_power_raw) > maxRawPower){ maxRawPower = Math.abs(br_power_raw);}
        if(Math.abs(tr_power_raw) > maxRawPower){ maxRawPower = Math.abs(tr_power_raw);}

        //if the maximum is greater than 1, scale all the powers down to preserve the shape
        double scaleDownAmount = 1.0;
        if(maxRawPower > 1.0){
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            scaleDownAmount = 1.0/maxRawPower;
        }
        tl_power_raw *= scaleDownAmount;
        bl_power_raw *= scaleDownAmount;
        br_power_raw *= scaleDownAmount;
        tr_power_raw *= scaleDownAmount;//test for commit to my old branch

        frontLeft.setPower(tl_power_raw);
        backLeft.setPower(bl_power_raw);
        backRight.setPower(br_power_raw);
        frontRight.setPower(tr_power_raw);


    }
}
