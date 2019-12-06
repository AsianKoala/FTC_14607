package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.opmodes;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Auto.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses.Auto;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses.TimeProfiler;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.*;
import static org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.RobotMovement.*;
import static org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.RobotPosition.*;


/**
 * just for copy and pasting quick new autos
 */
@Autonomous
public class AutoLayout extends Auto {

    public boolean safePark = true;

    public enum progStates {
        move,
        stop
    }



    @Override
    public void init() {
        super.init();


        // set starting pose
        giveMePose(blueFoundationStart);
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
        telemetry.addLine("pepeD");
        telemetry.addLine("a is true, b is false");
        telemetry.addData("safe parking?", safePark);
        telemetry.update();
    }


    @Override
    public void start() {
        super.start();
        giveMePose(blueFoundationStart);
        currStage = progStates.move.ordinal();
    }




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

        allTimeProfilers.get(2).markStart();
        scaledPositionTelemetry();
        allTimeProfilers.get(2).markEnd();


        telemetry.addLine("------------- AUTO PROFILER TELEM ---------");
        telemetry.addLine("loop profiler " + allTimeProfilers.get(0).getAverageTimePerUpdateMillis());
        telemetry.addLine("position telem profiler: " + allTimeProfilers.get(1).getAverageTimePerUpdateMillis());
        telemetry.addLine("scaled pos telem profiler: " + allTimeProfilers.get(2).getAverageTimePerUpdateMillis());
    }



    public void positionTelemetry() {
        telemetry.addLine("xPos: " + df.format(worldXPos) +
                "yPos: "+ df.format(worldYPos) +
                "heading: " + df.format(worldHeadingRad));
    }

    public void scaledPositionTelemetry() {
        telemetry.addLine("scaled xPos: " + df.format(scaledWorldXPos) + "scaled yPos: " + df.format(scaledWorldYPos) + "scaled heading: " + df.format(scaledWorldHeadingRad));
    }







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

        if(currStage == progStates.move.ordinal()) {
            if (stageFinished) {
                initStateVars();
                giveMePose(blueFoundationStart);
            }


            goToPosition(stageStartingXPos, stageStartingYPos + 24, stageStartingAngleRad, 1, 1);

            if(goToPositionCompleted) {
                nextStage();
            }

        }

        if(currStage == progStates.stop.ordinal()) {
            initStateVars();
            stopMovement();
            telemetry.addLine("shutting off in 5 seconds");
            if(timedOut(5000)) {
                requestOpModeStop();
            }
        }



    }
}
