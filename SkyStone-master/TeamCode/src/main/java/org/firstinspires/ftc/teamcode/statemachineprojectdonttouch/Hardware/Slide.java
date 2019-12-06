package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses.Firefly;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.psuedoHomer;

public class Slide {
    private ExpansionHubMotor leftSlide;
    private ExpansionHubMotor rightSlide;
    private Firefly myRobot;

    public ArrayList<ExpansionHubMotor> allMotors = new ArrayList<>();
    // TODO: change these values after you get them from opmode tuner
    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    private PIDCoefficients initCoeffs = new PIDCoefficients(P,I,D);

    private boolean isDebugging = true;
    private double newTarget;

    STATES states = STATES.HOME;
    private enum STATES {
        HOME,
        CUSTOM,
    }



    public Slide(Firefly myRobot, ExpansionHubMotor leftSlide, ExpansionHubMotor rightSlide) {
        this.myRobot = myRobot;
        this.leftSlide = leftSlide;
        this.rightSlide = rightSlide;
        allMotors.add(leftSlide);
        allMotors.add(rightSlide);



        for(ExpansionHubMotor expansionHubMotor : allMotors) {
            expansionHubMotor.setTargetPosition(0);
            expansionHubMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            expansionHubMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            expansionHubMotor.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, initCoeffs);
        }

        rightSlide.setDirection(DcMotor.Direction.REVERSE);

    }



    public void goPsuedoHome() {
        newTarget = psuedoHomer;
    }

    public void goCustom(double newTarget, boolean increment) {
        if(increment) {
            this.newTarget = newTarget + leftSlide.getCurrentPosition();
        }

        else {
            this.newTarget = newTarget;
        }
    }

    public void setDebugging(boolean debugging) {
        isDebugging = debugging;
    }






    public void update() {


        if(Math.abs(newTarget - leftSlide.getCurrentPosition()) > 10 || Math.abs(newTarget - rightSlide.getCurrentPosition()) > 10) {
            leftSlide.setTargetPosition((int)(newTarget));
            rightSlide.setTargetPosition((int)(newTarget));
            leftSlide.setPower(1);
            rightSlide.setPower(1);
        }

        else {
            leftSlide.setPower(0);
            rightSlide.setPower(0);
        }


        myRobot.addSpace();
        myRobot.telemetry.addLine("-------- SLIDE TELEM HERE ---------");
        myRobot.telemetry.addData("left slide curr pos", leftSlide.getCurrentPosition());
        myRobot.telemetry.addData("right slide curr pos", rightSlide.getCurrentPosition());
        if(isDebugging) {
            myRobot.telemetry.addData("current state", states);
            myRobot.telemetry.addData("LS motor target pos", leftSlide.getTargetPosition());
            myRobot.telemetry.addData("RS motor target pos", rightSlide.getTargetPosition());
            myRobot.telemetry.addData("PID GAINS", new PIDCoefficients(P,I,D));
        }
    }

}