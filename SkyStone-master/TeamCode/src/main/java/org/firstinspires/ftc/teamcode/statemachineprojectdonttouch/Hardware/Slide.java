package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.ArrayList;

public class Slide {
    private ExpansionHubMotor leftSlide;
    private ExpansionHubMotor rightSlide;

    public ArrayList<ExpansionHubMotor> allMotors = new ArrayList<>();
    // TODO: change these values after you get them from opmode tuner
    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    private PIDCoefficients initCoeffs = new PIDCoefficients(P,I,D);

    private boolean isDebugging = true;
    private int newPosition;
    private int oldPosition;

    STATES states = STATES.HOME;
    private enum STATES {
        PSEUDOHOME,
        HOME,
        CUSTOM,
        OUT,
        OLDPOS
    }



    public Slide(ExpansionHubMotor leftSlide, ExpansionHubMotor rightSlide) {
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




    private void setTargetPosition(int newPosition) {
        this.newPosition = newPosition;
    }

    private void applyPIDCoeffs() {
        leftSlide.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients(P,I,D));
        rightSlide.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients(P,I,D));
    }

    private void recordOldPos() {
        oldPosition = getLeftSlidePosition();
    }




    private void HandleMovements() {
        if(states == STATES.HOME) {
            recordOldPos();
            setTargetPosition(13);
        }

        if(states == STATES.PSEUDOHOME) {
            recordOldPos();
            setTargetPosition(25);
        }

        if(states == STATES.OUT) {
            setTargetPosition(-500);
        }

        if(states == STATES.OLDPOS) {
            setTargetPosition(oldPosition);
        }

        if(states == STATES.CUSTOM) {

        }
    }





    
    public double getNewPosition() {
        return newPosition;
    }

    public int getLeftSlidePosition() {
        return leftSlide.getCurrentPosition();
    }

    public int getRightSlidePosition() {
        return rightSlide.getCurrentPosition();
    }

    public void setPIDCoeffs(double p, double i, double d) {
        P = p;
        I = i;
        D = d;
    }


    public void setDebugging(boolean debugging) {
        isDebugging = debugging;
    }



    public void goHome() { states = STATES.HOME;}
    public void goPsuedohome() { states = STATES.PSEUDOHOME;}
    public void goOut() { states = STATES.OUT;}
  
    public void manualMovement(int newPosition, boolean isIncremental) {
        if(isIncremental){
            setTargetPosition(leftSlide.getCurrentPosition() + newPosition);
        }

        else {
            setTargetPosition(newPosition);
        }

        states = STATES.CUSTOM;
    }


    public void goOld() { states = STATES.OLDPOS; }





    public void update() {

        HandleMovements();

        if (Math.abs(leftSlide.getCurrentPosition() - newPosition) > 10 || Math.abs(rightSlide.getCurrentPosition() - newPosition) > 10) {

            if (isDebugging) {
                applyPIDCoeffs();
            }

            leftSlide.setTargetPosition( newPosition);
            rightSlide.setTargetPosition( newPosition);


            leftSlide.setPower(1);
            rightSlide.setPower(1);
        }

        else {
            leftSlide.setPower(0);
            rightSlide.setPower(0);
        }
    }

}
