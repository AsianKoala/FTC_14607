package org.firstinspires.ftc.teamcode.Auto.neils_corner;

import org.opencv.core.Point;

public class TestAuto extends BaseAuto {


    @Override
    public void runOpMode() {
        super.runOpMode();

        setStartingPosition(new Point(0,0));



        while(!isStarted() && !isStopRequested()) {

        }

        phoneCam.closeCameraDevice();


        goToPosition(0,24, 0, 0.5, 0.5,  Math.toRadians(15), true);
        goToPosition(10,24,0,0.5,0.5, Math.toRadians(15), true);
        goToPosition(10,24,Math.toRadians(90),0, 0.5, Math.toRadians(15), true);
    }

}
