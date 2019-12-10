package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses.Auto;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses.TimeProfiler;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.BetterRobotPosition;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.RobotMovement;

import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.RobotMovement.*;
import static org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.RobotPosition.*;
import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.ourSkystonePosition;

public class BlueAuto extends Auto {
    private enum progStates {
        firstMovement,
        secondMovement,
        thirdMovement,
        fourthMovement,
        fifthMovement,
        sixthMovement
    }



    private boolean completedMovement(double targetX, double targetY) {
        return Math.hypot(targetX - scaledWorldXPos, targetY - scaledWorldYPos) < 2;
    }

    private boolean completedCurveMovement(double targetX, double targetY) {
        return Math.hypot(targetX - scaledWorldXPos, targetY - scaledWorldYPos) < 5;
    }



    @Override
    public void init() {
        super.init();
        BetterRobotPosition.setPosition(35,80, Math.toRadians(90));
        giveMeScaledPos(new Pose2d(35, 80, toRadians(90)));
    }


    @Override
    public void init_loop() {
        super.init_loop();
    }


    @Override
    public void start() {
        super.start();
        BetterRobotPosition.setPosition(35, 80, Math.toRadians(90));
        giveMeScaledPos(new Pose2d(35, 80, Math.toRadians(90)));
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
        telemetry.addLine("position telem: " +BetterRobotPosition.worldXPosition + "," + BetterRobotPosition.worldYPosition + "," + BetterRobotPosition.worldAngleRad);
    }




    @Override
    public void MainStateMachine() {
        if (ourSkystonePosition == GLOBALS.SKYSTONE_POSITION.LEFT) {
            if (currStage == progStates.firstMovement.ordinal()) {
                if (stageFinished) {
                    initStateVars();
                    BetterRobotPosition.setPosition(35, 80, Math.toRadians(90));
                }


                gunToPosition(125, 97, 0, 1, 0, 0, 0, true);
                pointAngle(toRadians(55), 1, toRadians(10));


                if(completedMovement(125,97)) {
                    nextStage();
                }
            }


            if(currStage == progStates.secondMovement.ordinal()) {
                if(stageFinished) {
                    initStateVars();
                }


                gunToPosition(100,150,0,1,0,0,0,false);
                pointAngle(toRadians(90), 1, toRadians(10));


                if(completedCurveMovement(100, 150)) {
                    nextStage();
                }
            }


            if(currStage == progStates.thirdMovement.ordinal()) {
                if(stageFinished) {
                    initStateVars();
                }

                gunToPosition(100,290,0,0.25,0,0,0, true);
                pointAngle(0,0.5,toRadians(10));

                if(completedMovement(100,290)) {
                    nextStage();
                }

            }


            if(currStage == progStates.fourthMovement.ordinal()) {
                if(stageFinished) {
                    initStateVars();
                }

                gunToPosition(60,280, 0, 1, 0, 0,0,true);
                pointAngle(toRadians(90),1,toRadians(5));

                if(completedMovement(60,280)) {
                    nextStage();
                }
            }


            if(currStage == progStates.fifthMovement.ordinal()) {
                if(stageFinished) {
                    initStateVars();
                }

                gunToPosition(97,170,0,0.25,0,0,0,true);
                pointAngle(toRadians(-90),1,toRadians(10));

                if(completedMovement(97,170)) {
                    stopMovement();
                }
            }
        }



        else if(ourSkystonePosition == GLOBALS.SKYSTONE_POSITION.MIDDLE) {
            if(currStage == progStates.firstMovement.ordinal()) {
                if(stageFinished) {
                    initStateVars();
                }

                gunToPosition(125, 80, 0, 0.3, 0, 0, 0, true);
                pointAngle(toRadians(45), 1, toRadians(10));

                if(completedMovement(125,80)) {
                    nextStage();
                }
            }


            if(currStage == progStates.secondMovement.ordinal()) {
                if(stageFinished) {
                    initStateVars();
                }

                gunToPosition(97,150,0,1,0,0,0,false);
                pointAngle(toRadians(90), 0.3, toRadians(10));

                if(completedCurveMovement(97,150)) {
                    nextStage();
                }
            }


            if(currStage == progStates.thirdMovement.ordinal()) {
                if(stageFinished) {
                    initStateVars();
                }

                gunToPosition(95,290,0,0.25,0,0,0, true);
                pointAngle(0,0.5,toRadians(10));

                if(completedMovement(95,290)) {
                    nextStage();
                }
            }


            if(currStage == progStates.fourthMovement.ordinal()) {
                if(stageFinished) {
                    initStateVars();
                }

                gunToPosition(60,290, 0, 1, 0, 0,0,true);
                pointAngle(toRadians(90),1,toRadians(5));

                if(completedMovement(60,290)) {
                    nextStage();
                }
            }


            if(currStage == progStates.fifthMovement.ordinal()) {
                if(stageFinished) {
                    initStateVars();
                }

                gunToPosition(97,170,0,0.25,0,0,0,true);
                pointAngle(toRadians(-90),1, toRadians(10));

                if(completedMovement(97,170)) {
                    stopMovement();
                }
            }
        }



        else if(ourSkystonePosition == GLOBALS.SKYSTONE_POSITION.RIGHT) {
            if(currStage == progStates.firstMovement.ordinal()) {
                if(stageFinished) {
                    initStateVars();
                }

                gunToPosition(125, 60, 0, 0.5, 0, 0, 0, true);
                pointAngle(toRadians(40), 0.8, toRadians(10));

                if(completedMovement(125,60)) {
                    nextStage();
                }
            }


            if(currStage == progStates.secondMovement.ordinal()) {
                if(stageFinished) {
                    initStateVars();
                }

                gunToPosition(95,150,0,1,0,0,0,false);
                pointAngle(toRadians(90), 0.3, toRadians(10));

                if(completedCurveMovement(95,150)) {
                    nextStage();
                }
            }


            if(currStage == progStates.thirdMovement.ordinal()) {
                if(stageFinished) {
                    initStateVars();
                }

                gunToPosition(95,290,0,0.25,0,0,0, true);
                pointAngle(0,0.5,toRadians(10));

                if(completedMovement(95,290)) {
                    nextStage();
                }
            }


            if(currStage == progStates.fourthMovement.ordinal()) {
                if(stageFinished) {
                    initStateVars();
                }

                gunToPosition(60,280, 0, 1, 0, 0,0,true);
                pointAngle(toRadians(90),1,toRadians(5));

                if(completedMovement(60,280)) {
                    nextStage();
                }
            }


            if(currStage == progStates.fifthMovement.ordinal()) {
                if(stageFinished) {
                    initStateVars();
                }

                gunToPosition(97,175,0,0.25,0,0,0,true);
                pointAngle(toRadians(-90),1,toRadians(10));

                if(completedMovement(97,175)) {
                    stopMovement();
                }
            }
        }
    }
}
