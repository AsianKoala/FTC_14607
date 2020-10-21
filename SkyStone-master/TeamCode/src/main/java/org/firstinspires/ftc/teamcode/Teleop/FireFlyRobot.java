package org.firstinspires.ftc.teamcode.Teleop;
import android.annotation.SuppressLint;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;
import org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS;
import org.firstinspires.ftc.teamcode.HelperClasses.ppProject.company.Range;
import org.firstinspires.ftc.teamcode.HelperClasses.ppProject.company.Robot;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.*;

public class FireFlyRobot {

    private ExpansionHubMotor leftFront;
    private ExpansionHubMotor rightFront;
    private ExpansionHubMotor leftRear;
    private ExpansionHubMotor rightRear;
    public ExpansionHubMotor leftIntake;
    public ExpansionHubMotor rightIntake;
    public ExpansionHubMotor leftSlide;
    public ExpansionHubMotor rightSlide;
    public Servo clawRotater, clawGripper, clawFlipper, leftHook, rightHook, capstoneDeployer, parkingDeployer, horizontalExtend, frontGripper, backGripper;

    public void init(HardwareMap hwMap) {

        HardwareMap hardwareMap = null;
        hardwareMap = hwMap;

        leftFront = hardwareMap.get(ExpansionHubMotor.class, "FL");
        leftRear = hardwareMap.get(ExpansionHubMotor.class, "BL");
        rightRear = hardwareMap.get(ExpansionHubMotor.class, "FR");
        rightFront = hardwareMap.get(ExpansionHubMotor.class, "BR");
        leftIntake = hardwareMap.get(ExpansionHubMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(ExpansionHubMotor.class, "rightIntake");
        leftSlide = hardwareMap.get(ExpansionHubMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(ExpansionHubMotor.class, "rightSlide");

        clawRotater = hardwareMap.get(Servo.class, "clawRotater");
        clawGripper = hardwareMap.get(Servo.class, "clawGripper");
        clawFlipper = hardwareMap.get(Servo.class, "clawFlipper");
        leftHook = hardwareMap.get(Servo.class, "leftHook");
        rightHook = hardwareMap.get(Servo.class, "rightHook");
        capstoneDeployer = hardwareMap.get(Servo.class, "capstoneDeployer");
        parkingDeployer = hardwareMap.get(Servo.class, "parkingDeployer");
        horizontalExtend = hardwareMap.get(Servo.class, "horizontalExtend");
        frontGripper = hardwareMap.get(Servo.class, "frontGripper");
        backGripper = hardwareMap.get(Servo.class, "backGripper");


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        leftSlide.setDirection(DcMotor.Direction.REVERSE);

        // todo
//        rightSlide.setDirection(DcMotor.Direction.REVERSE);


        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setDriveMotorsNoVelMode();

    }

    public void initPositions() {
        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients(P,I,D));
        rightSlide.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients(P,I,D));

//        setHorizontalExtendHome();
        setBackGripperClosed();
        setFrontGripperOpen();
        setFoundationHookHome();
        setCapstoneHome();
        setParkingDeployerIn();
    }


    // DRIVETRAIN CONTROL
    public void setFLPower(double power) {
        leftFront.setPower(power);
    }
    public void setFRPower(double power) {
        rightFront.setPower(power);
    }
    public void setBLPower(double power) {
        leftRear.setPower(power);
    }
    public void setBRPower(double power) {
        rightRear.setPower(power);
    }
    public void setDriveStop() {
        setFLPower(0);
        setFRPower(0);
        setBLPower(0);
        setBRPower(0);
    }

    // INTAKE CONTROL
    public void setLeftIntakePower(double power) {
        leftIntake.setPower(power);
    }
    public void setRightIntakePower(double power) {
        rightIntake.setPower(power);
    }
    public void setIntakeStop() {
        setLeftIntakePower(0);
        setRightIntakePower(0);
    }

    // LIFT CONTROL
    public int liftTargetPosition = 0;
    long timeTargetLastSet = 0;
    public void setLiftTargetPosition(int encoderTicks, double power) {
        liftTargetPosition = (int) Range.clip(encoderTicks, GLOBALS.maxLiftHeightEncoderTicks, 0);
        leftSlide.setTargetPosition(liftTargetPosition);
        rightSlide.setTargetPosition(liftTargetPosition);
        leftSlide.setPower(power);
        rightSlide.setPower(power);
        timeTargetLastSet = System.currentTimeMillis();
    }
    public void setLiftTargetPositionForced(int encoderTicks, double power) {
        liftTargetPosition = -encoderTicks;
        leftSlide.setTargetPosition(liftTargetPosition);
        rightSlide.setTargetPosition(liftTargetPosition);
        leftSlide.setPower(power);
        rightSlide.setPower(power);
        timeTargetLastSet = System.currentTimeMillis();
    }
    public int getLiftTargetPosition() {
        return liftTargetPosition;
    }
    public int getLiftPosition() {
        return (leftSlide.getCurrentPosition()+rightSlide.getCurrentPosition())/2;
    }
    public void setLiftTargetPositionHome(double power) {
        setLiftTargetPosition(0, power);
    }
    public boolean checkLiftHome(int thresholdHome) {
        if(Math.abs(getLiftPosition()) < thresholdHome) {
            return true;
        }
        return false;
    }
    public void setLiftPositionTolerance(int tolerance) {
        leftSlide.setTargetPositionTolerance(tolerance);
        rightSlide.setTargetPositionTolerance(tolerance);
    }
    public boolean checkLiftStalledTime(long timeout) {
        return System.currentTimeMillis()-timeTargetLastSet > timeout;
    }
    public void setLiftStop() {
        leftSlide.setPower(0);
        rightSlide.setPower(0);
    }
    public void setLiftEmergencyDisable() {
        leftSlide.setMotorDisable();
        rightSlide.setMotorDisable();
    }
    public void setLiftEmergencyEnable() {
        leftSlide.setMotorEnable();
        rightSlide.setMotorEnable();
    }



    // SERVOS



    // HORIZONTAL EXTENSION CONTROL
    public void setHorizontalExtendHome(){
        horizontalExtend.setPosition(GLOBALS.horizontalExtendHome);
    }
    public void setHorizontalExtendOut(){
        horizontalExtend.setPosition(GLOBALS.horizontalExtendOut);
    }
    public void setHorizontalExtendFeed(){
        horizontalExtend.setPosition(GLOBALS.horizontalExtendFeed);
    }
    public void setHorizontalExtendManual(double position){
        horizontalExtend.setPosition(Range.clip(position, GLOBALS.horizontalExtendHome, GLOBALS.horizontalExtendOut));
    }
    public double getHorizontalExtendPosition() {
        return horizontalExtend.getPosition();
    }

    // FOUNDATION GRIPPER CONTROL
    public void setFoundationHookGrip() {
        leftHook.setPosition(GLOBALS.leftHookGrip);
        rightHook.setPosition(GLOBALS.rightHookGrip);
    }
    public void setFoundationHookHome() {
        leftHook.setPosition(GLOBALS.leftHookHome);
        rightHook.setPosition(GLOBALS.rightHookHome);
    }
    public boolean getFoundationHookState() {
        return leftHook.getPosition() == GLOBALS.leftHookGrip;
    }

    // CAPSTONE CONTROL
    public void setCapstoneOut() {
        capstoneDeployer.setPosition(GLOBALS.capstoneDeployerOut);
    }
    public void setCapstoneHome() {
        capstoneDeployer.setPosition(GLOBALS.capstoneDeployerHome);
    }
    public double getCapstonePosition() {
        return capstoneDeployer.getPosition();
    }
    public void setCapstonePosition(double position) {
        capstoneDeployer.setPosition(Range.clip(position, capstoneDeployerOut, capstoneDeployerHome));
    }

    // GRIPPERS CONTROL
    public void setFrontGripperOpen() {
        frontGripper.setPosition(GLOBALS.frontGripperOpen);
    }
    public void setFrontGripperClosed() {
        frontGripper.setPosition(GLOBALS.frontGripperClosed);
    }
    public double getFrontGripperPosition() {
        return frontGripper.getPosition();
    }
    public void setBackGripperOpen() {
        backGripper.setPosition(GLOBALS.backGripperOpen);
    }
    public void setBackGripperClosed() {
        backGripper.setPosition(GLOBALS.backGripperClosed);
    }
    public void setBackGripperCap() {
        backGripper.setPosition(GLOBALS.backGripperCap);
    }
    public double getBackGripperPosition() {
        return backGripper.getPosition();
    }

    // PARKING DEPLOYER CONTROL
    public void setParkingDeployerOut() {
        parkingDeployer.setPosition(GLOBALS.parkingDeployerOut);
    }
    public void setParkingDeployerIn() {
        parkingDeployer.setPosition(GLOBALS.parkingDeployerIn);
    }
    public void setParkingDeployerPosition(double position) {
        parkingDeployer.setPosition(Range.clip(position, GLOBALS.parkingDeployerOut, GLOBALS.parkingDeployerIn));
    }
    public double getParkingDeployerPosition() {
        return parkingDeployer.getPosition();
    }

    // COMPOUND COMMANDS
//    public void

    public void setDriveMotorsVelocityMode() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients(25, 0, 0.5));
        leftRear.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients(25, 0, 0.5));
        rightFront.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients(25, 0, 0.5));
        rightRear.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients(25, 0, 0.5));
    }

    public void setDriveMotorsNoVelMode() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}