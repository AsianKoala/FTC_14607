//package org.firstinspires.ftc.teamcode.Teleop;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.PIDCoefficients;
//import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;
//import org.openftc.revextensions2.ExpansionHubMotor;
//import org.openftc.revextensions2.ExpansionHubServo;
//
//import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.*;
//
//public class LiftCurrentTuner extends OpMode
//{
//    private ExpansionHubMotor leftSlide;
//    private ExpansionHubMotor rightSlide;
//
//    public void init(){
//        leftSlide = hardwareMap.get(ExpansionHubMotor.class, "leftSlide");
//        rightSlide = hardwareMap.get(ExpansionHubMotor.class, "rightSlide");
//    }
//
//    public void loop() {
//        for(int i = 0; i < 100; i += 10){
//            leftSlide.setPosition(i);
//            rightSlide.setPosition(i);
//            telemetry.addData("Left Slide current", leftSlide.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
//            telemetry.addData("Right Slide current", rightSlide.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
//            delay(2000);
//        }
//    }
//
//    public void delay(int millis) {
//        long now = System.currentTimeMillis();
//        while(System.currentTimeMillis() - now < millis){
//
//        }
//
//    }
//
//}
