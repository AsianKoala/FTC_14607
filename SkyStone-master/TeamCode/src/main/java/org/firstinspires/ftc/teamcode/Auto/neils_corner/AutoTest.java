package org.firstinspires.ftc.teamcode.Auto.neils_corner;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * extending base auto for localization methods
 */
@TeleOp
public class AutoTest extends BaseAuto {

    @Override
    public void runOpMode() {





        while(!isStarted()) {

        }

        phoneCam.closeCameraDevice();

        verticalMovement(48, 0.5, 0.25, 4, 4, 0.75, Math.toRadians(20));
    }
}
