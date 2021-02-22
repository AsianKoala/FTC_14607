package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.control.Robot;
import org.openftc.revextensions2.ExpansionHubServo;

import java.util.ArrayList;

@TeleOp(name = "Servo Programmer")
public class ServoProgrammer extends Robot {
    public ServoData ourServoData;

    @Override
    public void init() {
        super.init();
        ExpansionHubServo servo = hardwareMap.get(ExpansionHubServo.class, "leftPivot"); // ex
        ourServoData = new ServoData(servo, 0.05);
    }

    @Override
    public void loop() {
        super.loop();
        try {
            sleep();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        telemetry.addLine(ourServoData.update(gamepad1.x, gamepad1.b, gamepad1.a));
    }


    private void sleep() throws InterruptedException {
        Thread.sleep(300);
    }
}

class ServoData {
    private final ExpansionHubServo servo;
    private double val;
    private final double increment;
    ServoData(ExpansionHubServo servo, double increment) {
        this.servo = servo;
        val = 0.5;
        this.increment = increment;
    }

    public String update(boolean isDecrease, boolean isIncrease, boolean set) {
        if(isDecrease) {
            val = Range.clip(val - increment, 0, 1);
        } else if(isIncrease) {
            val = Range.clip(val + increment, 0, 1);
        }

        if(set) {
            servo.setPosition(val);
        }

        return servo.getDeviceName() + ": " + val;
    }
}