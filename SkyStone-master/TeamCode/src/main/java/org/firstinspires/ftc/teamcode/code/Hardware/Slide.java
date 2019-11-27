package org.firstinspires.ftc.teamcode.code.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.ArrayList;

public class Slide {
    public ExpansionHubMotor leftSlide;
    public ExpansionHubMotor rightSlide;
    public ArrayList<ExpansionHubMotor> allMotors = new ArrayList<>();
    private double P, I, D;
    private boolean isDebugging;




    public Slide(ExpansionHubMotor leftSlide, ExpansionHubMotor rightSlide) {
        this.leftSlide = leftSlide;
        this.rightSlide = rightSlide;
        allMotors.add(leftSlide);
        allMotors.add(rightSlide);

        for(ExpansionHubMotor expansionHubMotor : allMotors) {
            expansionHubMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            expansionHubMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        rightSlide.setDirection(DcMotor.Direction.REVERSE);
    }










    public void setPIDCoeffs(PIDCoefficients coeffs) {
        leftSlide.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, coeffs);
        rightSlide.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, coeffs);
    }


    public PIDCoefficients getPIDCoeffs() {
        return leftSlide.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
    }




    public void setTargetPosition(int position) {
        leftSlide.setTargetPosition(position);
        rightSlide.setTargetPosition(position);
    }


    public void setPower(double powers) {
        leftSlide.setPower(powers);
        rightSlide.setPower(powers);
    }


    public boolean isDebugging() {
        return isDebugging;
    }

    public boolean setDebug(boolean state) {
        isDebugging = state;
        return isDebugging;
    }

}
