package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.opmodes;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.*;

import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.Auto.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses.Auto;

public class OpenCVTestAuto extends Auto {

    private enum progStates {
        move,
        stop
    }


    public void init() {
        super.init();
    }

    public void init_loop() {
        super.init_loop();
    }

    public void start() {
        super.start();
    }

    public void loop() {
        super.loop();
    }



    @Override
    public void MainStateMachine() {


        if(currStage == progStates.move.ordinal()) {
            if(stageFinished) {
                initStateVars();

                switch(ourSkystonePosition) {
                    case LEFT:
                        Trajectory left = new TrajectoryBuilder(stageStartingPose, DriveConstants.BASE_CONSTRAINTS)
                                .strafeLeft(24)
                                .build();
                        myDriveTrain.followTrajectory(left);
                    case MIDDLE:
                        Trajectory middle = new TrajectoryBuilder(stageStartingPose, DriveConstants.BASE_CONSTRAINTS)
                                .forward(24)
                                .build();
                    case RIGHT:
                        Trajectory right = new TrajectoryBuilder(stageStartingPose, DriveConstants.BASE_CONSTRAINTS)
                                .strafeRight(24)
                                .build();
                }



                if(!myDriveTrain.isBusy()) {
                    nextStage();
                }
            }
        }


        if(currStage == progStates.stop.ordinal()) {
            myDriveTrain.setMotorPowers(0,0,0,0);
            telemetry.addLine("requesting opmode stop in " + (stateStartTime - currTimeMillis) +" seconds");
            if(timedOut(5000)) {
                requestOpModeStop();
            }
        }
    }
}
