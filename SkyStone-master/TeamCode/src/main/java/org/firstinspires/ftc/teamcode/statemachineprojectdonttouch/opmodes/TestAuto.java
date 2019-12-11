package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses.Auto;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses.TimeProfiler;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.BetterRobotPosition;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.RobotMovement;

import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.RobotMovement.*;
import static org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.RobotPosition.*;
import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.ourSkystonePosition;

@Autonomous(name = "test ato")
public class TestAuto extends Auto {
    private enum progStates {

        firstMovement,
        secondMovement,
        thirdMovement,
        fourthMovement,
        fifthMovement,
        sixthMovement
    }



    private boolean completedMovement(double targetX, double targetY) {
        return Math.hypot(targetX - worldXPos, targetY - worldYPos) < 2;
    }

    private boolean completedCurveMovement(double targetX, double targetY) {
        return Math.hypot(targetX - worldXPos, targetY - worldYPos) < 5;
    }



    @Override
    public void init() {
        super.init();

    }


    @Override
    public void init_loop() {
        super.init_loop();
    }


    @Override
    public void start() {
        super.start();
       giveMePose(new Pose2d(0,0,0));
        currStage = progStates.firstMovement.ordinal();
    }



    TimeProfiler loopProfiler = new TimeProfiler(500);

    @Override
    public void loop() {
        loopProfiler.markStart();
        super.loop();
        loopProfiler.markEnd();


        addSpace();
        telemetry.addLine("--------------------- AUTO TELEM --------------");
        allPositionTelemetry();
        telemetry.addData("current progState", currStage);
    }

    public void allPositionTelemetry() {
        telemetry.addData("rr world x pos ", worldXPos);
        telemetry.addData("rr world y pos ", worldYPos);
        telemetry.addData("rr world heading ", Math.toDegrees(worldHeadingRad));
        addSpace();
        telemetry.addData("scaled world x pos ", scaledWorldXPos);
        telemetry.addData("scaled world y pos ", scaledWorldYPos);
        telemetry.addData("scaled world heading ", Math.toDegrees(BetterRobotPosition.worldAngleRad));
    }




    @Override
    public void MainStateMachine() {
        if (ourSkystonePosition == GLOBALS.SKYSTONE_POSITION.LEFT) {
            if (currStage == progStates.firstMovement.ordinal()) {
                if (stageFinished) {
                    giveMePose(new Pose2d(0, 0, 0));
                    initStateVars();
                }

                goToRRPosition(stageStartingXPos, stageStartingYPos + 24, 0, 0.5, 0.5);

                if (completedMovement(stageStartingXPos, stageStartingYPos + 24)) {
                    nextStage();
                }
            }

            if(currStage == progStates.secondMovement.ordinal()) {
                if(stageFinished) {
                    initStateVars();
                }

                pointRRAngle(Math.toRadians(90), 0.5, Math.toRadians(30));

                if(stageStartingAngleRad - worldHeadingRad < Math.toRadians(3)) {
                    requestOpModeStop();
                }
            }

            if(currStage == progStates.thirdMovement.ordinal()) {
                if(stageFinished) {
                    initStateVars();
                }

                goToRRPosition(stageStartingXPos + 12, stageStartingYPos + 24, Math.toRadians(45), 0.5, 0.5);

                if(completedMovement(stageStartingXPos + 12, stageStartingYPos + 24)) {
                    nextStage();
                }
            }

            if(currStage == progStates.fourthMovement.ordinal()) {
                if(stageFinished) {
                    initStateVars();
                }

                gunToRRPosition(stageStartingXPos - 12, stageStartingYPos - 24, 0, 0.5, 0, 0, 0, true);
                pointRRAngle(Math.toRadians(0), 0.5, Math.toRadians(15));

                if(completedMovement(stageStartingXPos -12, stageStartingYPos - 24)) {
                    requestOpModeStop();
                }

            }
        }
    }
}