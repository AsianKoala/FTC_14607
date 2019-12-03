package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses;

import android.os.SystemClock;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import static org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.RobotPosition.*;



/**
 * base class for auto opmodes
 */
public abstract class Auto extends org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses.Firefly {

    public double stateStartTime = 0;
    public int currStage = 0; // current stage
    int nextStage = 1; // holds next stage
   public  boolean stageFinished = true; // used to progress to next stage

    public void nextStage(int ordinal) {
        nextStage = ordinal;
        incrementStage();
    }

    public void nextStage() {
        nextStage(currStage + 1);
    }

    public void incrementStage() {
        currStage = nextStage;
        stageFinished = true;
    }


    /**
     * use for debugging
     * @param millis time
     * @return
     */
    public boolean timedOut(int millis) {
        return (currTimeMillis - stateStartTime > millis);
    }



    public double stageStartingXPos = 0;
    public double stageStartingYPos = 0;
    public double stageStartingAngleRad = 0;
    public Pose2d stageStartingPose = new Pose2d(0,0,0);

    public void initStateVars() {
        stageFinished = false;
        stageStartingXPos = worldXPos;
        stageStartingYPos = worldYPos;
        stageStartingAngleRad = worldHeadingRad;
        stageStartingPose = new Pose2d(stageStartingXPos, stageStartingYPos, stageStartingAngleRad);
        stateStartTime = currTimeMillis;
    }


    
    
    

    private Pose2d startingPose;

    public void initPosition(Pose2d pose) {
        startingPose = pose;
    }



    @Override
    public void init() {
        super.init();
        myDetector.initCamera();
    }


    /**
     * sets our position to the starting position
     */
    @Override
    public void init_loop() {
        super.init_loop();
        //set position
        setPose(startingPose);
        myDetector.update();
    }


    @Override
    public void start() {
        super.start();
        // set position again
        setPose(startingPose);

        stageFinished = true;
        currStage = 0;
        currTimeMillis = SystemClock.uptimeMillis();
    }

    @Override
    public void loop() {
        super.loop();
        MainStateMachine();
    }

    // override this
    public abstract void MainStateMachine();

}

