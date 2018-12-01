package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


/**
 * Created by charliewu on 9/20/18.
 */

public class HardwareDragonfly {
    /* Public OpMode members. */
    public DcMotor fl   = null;
    public DcMotor  fr  = null;
    public DcMotor bl   = null;
    public DcMotor  br  = null;

    public DcMotor arm   = null;
    public DcMotor cascade   = null;
    public DcMotor lift   = null;

    public CRServo intake = null;

    public Servo intakeDoor = null;
    public Servo hangRelease = null;
    public Servo markerDeployer = null;
    public Servo hookRelease = null;

    public BNO055IMU revIMU = null;

//
//    public Servo sv1 = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareDragonfly(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        double sp = 0;

//        sv1 = hwMap.servo.get("servo1");

        // Define and Initialize Motors
        fl   = hwMap.dcMotor.get("fl");
        fr  = hwMap.dcMotor.get("fr");
        bl   = hwMap.dcMotor.get("bl");
        br  = hwMap.dcMotor.get("br");

        arm   = hwMap.dcMotor.get("arm");
        lift   = hwMap.dcMotor.get("lift");
        cascade   = hwMap.dcMotor.get("cascade");

        intake = hwMap.crservo.get("intake");

        intakeDoor = hwMap.servo.get("intakeDoor");
        hangRelease = hwMap.servo.get("hangRelease");
        markerDeployer = hwMap.servo.get("markerDeployer");
        hookRelease = hwMap.servo.get("hookRelease");

        revIMU = hwMap.get(BNO055IMU.class, "revIMU");
        revIMU.initialize(new BNO055IMU.Parameters());

        fl.setPower(sp);
        fr.setPower(sp);
        bl.setPower(sp);
        br.setPower(sp);
        arm.setPower(sp);
        lift.setPower(sp);
        cascade.setPower(sp);

        intake.setPower(0.01);

        intakeDoor.setPosition(0.6);
        hangRelease.setPosition(0.2); // latch hang on start
        markerDeployer.setPosition(0.85);
        hookRelease.setPosition(0.6);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cascade.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cascade.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

    }
//
//    public void turnServoContinuous(double speed){
//        sv1.setPosition(speed);
//    }
//    public void stopServoContinuous(){
//        sv1.setPosition(0);
//    }

    public void resetEncoders()
    {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cascade.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetDriveEncoders()
    {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
//    public void encoderDrive(double flspeed, double frspeed, double blspeed, double brspeed, int fltarget, int frtarget, int bltarget, int brtarget) {
//        int newFLTarget;
//        int newFRTarget;
//        int newBLTarget;
//        int newBRTarget;
//
//        // Determine new target position, and pass to motor controller
//        newFLTarget = fl.getCurrentPosition()+fltarget;
//        newFRTarget = fr.getCurrentPosition()+frtarget;
//        newBLTarget = bl.getCurrentPosition()+bltarget;
//        newBRTarget = br.getCurrentPosition()+brtarget;
//
//        resetEncoders();
//
//        fl.setPower(flspeed);
//        fr.setPower(frspeed);
//        bl.setPower(blspeed);
//        br.setPower(brspeed);
//
//        // keep looping while we are still active, and there is time left, and both motors are running.
//        while(Math.abs(fl.getCurrentPosition())<Math.abs(fltarget) || Math.abs(fr.getCurrentPosition())<Math.abs(frtarget) || Math.abs(bl.getCurrentPosition())<Math.abs(bltarget) || Math.abs(br.getCurrentPosition())<Math.abs(brtarget))
//        {
//            if(!(Math.abs(fl.getCurrentPosition())<Math.abs(fltarget)))
//            {
//                fl.setPower(0);
//            }
//            if(!(Math.abs(fr.getCurrentPosition())<Math.abs(frtarget)))
//            {
//                fr.setPower(0);
//            }
//            if(!(Math.abs(bl.getCurrentPosition())<Math.abs(bltarget)))
//            {
//                bl.setPower(0);
//            }
//            if(!(Math.abs(br.getCurrentPosition())<Math.abs(brtarget)))
//            {
//                br.setPower(0);
//            }
//        }
//
//        fl.setPower(0);
//        fr.setPower(0);
//        bl.setPower(0);
//        br.setPower(0);
//    }


    public void driveLimitless(double left, double right) {
        fl.setPower(left);
        fr.setPower(right);
        bl.setPower(left);
        br.setPower(right);
    }

    public void allStop()
    {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public int getHeading(){
        return -(int)Math.floor(revIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }



}
