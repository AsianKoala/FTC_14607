package org.firstinspires.ftc.teamcode.code.Teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;
import org.firstinspires.ftc.teamcode.code.Hardware.Slide;
import org.openftc.revextensions2.ExpansionHubMotor;

@TeleOp
public class SlidePIDTuner extends TunableOpMode {
    Slide mySlide;
    public double p;
    public double i;
    public double d;

    @Override
    public void init() {
        ExpansionHubMotor leftSlideMotor = ((ExpansionHubMotor) hardwareMap.get("leftSlide"));
        ExpansionHubMotor rightSlideMotor = ((ExpansionHubMotor) hardwareMap.get("rightSlide"));

        mySlide = new Slide(leftSlideMotor, rightSlideMotor);
        mySlide.setDebugging(true);
    }


    @Override
    public void loop() {
        p = getInt("p");
        i = getInt("i");
        d = getInt("d");


        mySlide.setPIDCoeffs(p,i,d);



        double increment = gamepad2.right_stick_y * 100;
        if(Math.abs(increment) > 25) {
            mySlide.manualMovement(increment, true);
        }

        if(gamepad2.x) {
            mySlide.goPsuedohome();
        }

        if(gamepad2.b) {
            mySlide.goHome();
        }




        mySlide.update();



        telemetry.addData("P:  ", p);
        telemetry.addData("I:  ", i);
        telemetry.addData("D   ", d);
        telemetry.addData("left slide position", mySlide.leftSlide.getCurrentPosition());
        telemetry.addData("right slide position", mySlide.rightSlide.getCurrentPosition());
        telemetry.addData("target position", mySlide.getNewPosition());
    }
}
