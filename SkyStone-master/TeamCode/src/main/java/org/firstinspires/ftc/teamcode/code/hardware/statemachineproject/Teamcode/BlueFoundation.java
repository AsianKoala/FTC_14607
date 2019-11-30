package org.firstinspires.ftc.teamcode.code.hardware.statemachineproject.Teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.code.hardware.statemachineproject.HelperClasses.Auto;
import org.firstinspires.ftc.teamcode.code.hardware.statemachineproject.HelperClasses.TimeProfiler;
import org.firstinspires.ftc.teamcode.code.hardware.statemachineproject.RobotUtil.RobotPosition;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.drive.DriveConstants;

import static org.firstinspires.ftc.teamcode.code.hardware.statemachineproject.RobotUtil.RobotPosition.*;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.code.GLOBALS.*;


@Autonomous
public class BlueFoundation extends Auto {

    public boolean safePark = false;

    public enum progStates {
        driveToFoundation,
        grabbing,
        pulling,
        ungrab,

        park,
        safePark // used if we need to park safely
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

        telemetry.addLine("hi dev charlie krish reevu");
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
        telemetry.addLine("xPos: " + df.format(RobotPosition.worldXPos) +
                "yPos: "+ df.format(RobotPosition.worldYPos) +
                "heading: " + df.format(RobotPosition.worldHeadingRad));
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
                RobotPosition.setPose(blueFoundationStart);
            }
            initStateVars();


            Trajectory toFoundation = new TrajectoryBuilder(stageStartingPose, DriveConstants.BASE_CONSTRAINTS)
                    .forward(Math.abs(stageStartingYPos - blueFoundation.getY()))
                    .build();

            myDriveTrain.followTrajectory(toFoundation);

            if (!myDriveTrain.isBusy()) {
                nextStage();
            }
        }

        // add more stges here but you get it
    }
}
