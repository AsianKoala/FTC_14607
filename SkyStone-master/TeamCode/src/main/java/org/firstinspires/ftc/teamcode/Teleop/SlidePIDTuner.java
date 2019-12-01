package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.ArrayList;

@TeleOp(name = "slide pid tuner non restructure", group = "")
public class SlidePIDTuner extends TunableOpMode {
    private ExpansionHubMotor leftSlide;
    private ExpansionHubMotor rightSlide;
    private ArrayList<ExpansionHubMotor> allMotors = new ArrayList<>();
    
    private double P = 0;
    private double I = 0;
    private double D = 0;
    private double newSlidePosition;

//
    
    @Override
    public void init() {
        leftSlide = hardwareMap.get(ExpansionHubMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(ExpansionHubMotor.class, "rightSlide");
        
        allMotors.add(leftSlide);
        allMotors.add(rightSlide);
        
        for(ExpansionHubMotor dcMotorEx : allMotors) {
            dcMotorEx.setTargetPosition(0);
            dcMotorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            dcMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            dcMotorEx.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients(0,0,0));
        }

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("starting PID at: " +P+", " + I + ", " + D);
        telemetry.update();
    }
    
    @Override
    public void start() {
        
    }
    
    @Override
    public void loop() {

        P = getDouble("P");
        I = getDouble("I");
        D = getDouble("D");

        leftSlide.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients(P,I,D));
        rightSlide.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients(P,I,D));

        double increment = gamepad2.right_stick_y * 100;
        if(Math.abs(increment) > 25) {
            newSlidePosition = leftSlide.getCurrentPosition() + increment;
        }
        if(gamepad2.x) {
            newSlidePosition = -25;
        }

        if(gamepad2.b) {
            newSlidePosition = -25.0/2;
        }
        if(gamepad2.dpad_up) {
            newSlidePosition = -300;
        }





        if(Math.abs(newSlidePosition - leftSlide.getCurrentPosition()) > 10 || Math.abs(newSlidePosition - rightSlide.getCurrentPosition()) > 10) {
            leftSlide.setTargetPosition((int)(newSlidePosition));
            rightSlide.setTargetPosition((int)(newSlidePosition));
            leftSlide.setPower(1);
            rightSlide.setPower(1);
        }

        else {
            leftSlide.setPower(0);
            rightSlide.setPower(0);
        }




        telemetry.addData("left slide position", leftSlide.getCurrentPosition());
        telemetry.addData("right slide position", rightSlide.getCurrentPosition());
        telemetry.addData("target position", newSlidePosition);
        telemetry.addData("P", P);
        telemetry.addData("I", I);
        telemetry.addData("D", D);

    }
}
