package org.firstinspires.ftc.teamcode.lastMinutescuffedPP.Hardware;

import android.os.SystemClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.lastMinutescuffedPP.Movement.MovementVars.*;

/**
 * THIS CLASS IS FINISHED
 */
public class DriveTrain {
    public RevMotor topLeft;
    public RevMotor topRight;
    public RevMotor bottomLeft;
    public RevMotor bottomRight;
    public BNO055IMU imu;


    public DriveTrain(RevMotor tl, RevMotor tr, RevMotor bl, RevMotor br, BNO055IMU imuwu) {
        topLeft = tl;
        topRight = tr;
        bottomRight = bl;
        bottomRight = br;
        imu = imuwu;
        
        /** since we're using motors with encoders we have to set this to using encoders */
        // TODO: ONCE WE GET ODOMETRY SET CHANGE THIS STUFF
        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: CHANGE THIS AFTER COMP PLEASE
    }

    private long lastUpdateTime = 0;

    public double getCurrentAvgLeft() {
        return (double)(topLeft.getCurrentPosition() + bottomLeft.getCurrentPosition())/2;
    }

    public double getCurrentAvgRight() {
        return (double)(topRight.getCurrentPosition() + bottomRight.getCurrentPosition())/2;
    }

    /**
     * so since we arent using odometry afaik we cant track position of the robot after it strafes, so that sucks
     * this will probably cause a shitload of errors with movement but you know idc
     * well i do care
     * its more like having odometry is pretty important/the only damn way we can use the r OMEGALUL bot
     * @return
     */



    public void ApplyMovement() {
        /**
         * This makes it so we don't constantly update the robot movement like every second, which would lead to lag
         */
        long currTime = SystemClock.uptimeMillis();
        if(currTime - lastUpdateTime < 16) {
            return;
        }
        lastUpdateTime = currTime;

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


        //now we can set the powers ONLY IF THEY HAVE CHANGED TO AVOID SPAMMING USB COMMUNICATIONS
        topLeft.setPower(tl_power_raw);
        bottomLeft.setPower(bl_power_raw);
        bottomRight.setPower(br_power_raw);
        topRight.setPower(tr_power_raw);

    }
}
