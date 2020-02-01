package org.firstinspires.ftc.teamcode.Auto.neils_corner;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * extending base auto for localization methods
 */
@TeleOp
public class LocalizationTest extends BaseAuto {

    @Override
    public void runOpMode() {



        while(opModeIsActive())
        gamepadControl();
    }
}
