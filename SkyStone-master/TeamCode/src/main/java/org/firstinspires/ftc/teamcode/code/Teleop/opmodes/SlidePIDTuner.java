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
    }


    @Override
    public void loop() {
        p = getInt("p");
        i = getInt("i");
        d = getInt("d");


        PIDCoefficients coefficients = new PIDCoefficients(p,i,d);

        mySlide.setPIDCoeffs(coefficients);

        if(gamepad1.left_bumper) mySlide.setTargetPosition(-500);
        if(gamepad1.right_bumper) mySlide.setTargetPosition(0);



        mySlide.setPower(1);


        telemetry.addData("PID COEFFS", coefficients.toString());
        telemetry.addData("left slide position", mySlide.leftSlide.getCurrentPosition());
        telemetry.addData("right slide position", mySlide.rightSlide.getCurrentPosition());
    }
}
