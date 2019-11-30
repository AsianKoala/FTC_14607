package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.opmodes;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses.Auto;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses.TimeProfiler;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.code.HelperClasses.GLOBALS.*;
import static org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.RobotPosition.*;


@Autonomous
public class BlueFoundation extends Auto {

    public boolean safePark = false;

    public enum progStates {
        driveToFoundation,
        grabbing,
        pulling,
        ungrab,

        park,
        safePark, // used if we need to park safely
        stop
    }



    @Override
    public void init() {
        super.init();


        // set starting position at left foundation starting position
        initPosition(blueFoundationStart);
    }


    @Override
    public void init_loop() {
        super.init_loop();

        if(gamepad1.a) {
            safePark = true;
        }

        if(gamepad1.b) {
            safePark = false;
        }
        telemetry.addLine("hi dev charlie krish reevu");
        telemetry.addLine("a is true, b is false");
        telemetry.addData("safe parking?", safePark);
        telemetry.update();
    }


    @Override
    public void start() {
        super.start();

        currStage = progStates.driveToFoundation.ordinal();
    }



    // i literally jsut realized you can just make an array of timeprofilers instead of declaring each one
    // i am spe(e)d
    private static ArrayList<TimeProfiler> allTimeProfilers = new ArrayList<>();

    static {
        for(int i=1; i<=10; i++) {
            allTimeProfilers.add(new TimeProfiler(500));
        }
    }


    @Override
    public void loop() {

        allTimeProfilers.get(0).markEnd();
        allTimeProfilers.get(0).markStart();
        telemetry.addData("UPS: ", allTimeProfilers.get(0).getAverageTimePerUpdateMillis());


        telemetry.addData("PROG STAGE", currStage);


        allTimeProfilers.get(0).markStart();
        super.loop();
        allTimeProfilers.get(0).markEnd();


        allTimeProfilers.get(1).markStart();
        positionTelemetry();
        allTimeProfilers.get(1).markEnd();

        telemetry.addLine("auto profiler 0: " + allTimeProfilers.get(0).getAverageTimePerUpdateMillis());
        telemetry.addLine("auto profiler 1: " + allTimeProfilers.get(1).getAverageTimePerUpdateMillis());


    }



    public void positionTelemetry() {
        telemetry.addLine("xPos: " + df.format(worldXPos) +
                "yPos: "+ df.format(worldYPos) +
                "heading: " + df.format(worldHeadingRad));
    }


    /**
     * this is the huge chunky boi
     * tfw when you spent all thanksgiving break coding
     * time to see if roadrunner works well with state machines or not
     * also this input lag might be bad (?)
     * idk
     * lets see
     */


    @Override
    public void MainStateMachine() {
        /**
         * this is the basic layout of the flow
         * if(stageFinished) { // runs once when looping
         *      initStateVars(); // inits vars
         * }
         *
         *do whatever you want to do here
         *
         * if( done with thing || timed out) {    // this is our condition going to the next stage
         *      nextStage();
         * }
         *
         * **remember, do not loop anything in here, main state machine is looped**
         */

        if(currStage == progStates.driveToFoundation.ordinal()) {
            if (stageFinished) {
                setPose(blueFoundationStart);
                initStateVars();

                Trajectory toFoundation = new TrajectoryBuilder(stageStartingPose, DriveConstants.BASE_CONSTRAINTS)
                        .forward(Math.abs(stageStartingYPos - blueFoundation.getY()))
                        .build();

                myDriveTrain.followTrajectory(toFoundation);
            }




            if (!myDriveTrain.isBusy()) {
                nextStage();
            }
        }

        if(currStage == progStates.grabbing.ordinal()) {
            if(stageFinished) {
                initStateVars();
            }

            myOuttake.grabFoundation();

            if(timedOut(1000)) {
                nextStage();
            }
        }

        if(currStage == progStates.pulling.ordinal()) {
            if(stageFinished) {
                initStateVars();
                Trajectory toWall = new TrajectoryBuilder(stageStartingPose, DriveConstants.BASE_CONSTRAINTS)
                        .back(Math.abs(stageStartingYPos - blueFoundation.getY()))
                        .build();
                myDriveTrain.followTrajectory(toWall);
            }


            if(!myDriveTrain.isBusy()) {
                nextStage();
            }
        }

        if(currStage == progStates.ungrab.ordinal()) {
            if(stageFinished) {
                initStateVars();
            }

            myOuttake.ungrabFoundation();

            if(timedOut(1000)) {
                if(!safePark) {
                    nextStage();
                }

                else {
                    nextStage(progStates.safePark.ordinal());
                }
            }
        }

        if(currStage == progStates.park.ordinal()) {
            if(stageFinished) {
                initStateVars();

                Trajectory toPark = new TrajectoryBuilder(stageStartingPose, DriveConstants.BASE_CONSTRAINTS)
                        .strafeRight(Math.abs(stageStartingXPos - blueNotSafePark.getX()))
                        .build();
                myDriveTrain.followTrajectory(toPark);
            }


            if(!myDriveTrain.isBusy()) {
                nextStage(progStates.stop.ordinal());
            }
        }



        if(currStage == progStates.park.ordinal()) {
            if(stageFinished) {
                initStateVars();

            }
            int i = 0;
            if(30 * 1000 - currTimeMillis > 10 && i==0) { // the errors are stupid, main state machine is looped so i will be bigger than 0
                Trajectory toSafePark = new TrajectoryBuilder(stageStartingPose, DriveConstants.BASE_CONSTRAINTS)
                        .strafeRight(Math.abs(stageStartingXPos - 18))
                        .splineTo(blueSafePark)
                        .build();

                myDriveTrain.followTrajectory(toSafePark);
                i++;
            }

            if(!myDriveTrain.isBusy() && i==1) {
                nextStage(progStates.stop.ordinal());
            }
        }

        if(currStage == progStates.stop.ordinal()) {
            myDriveTrain.setMotorPowers(0,0,0,0);
        }




    }
}
