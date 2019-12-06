package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses;

import android.os.SystemClock;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.Hardware.DriveTrain;
import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.*;
import static org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.RobotPosition.*;



/**
 * base class for auto opmodes
 */
@Autonomous(name = "auto statemachine")
public class Auto extends Firefly {

    private DriveTrain thisDriveTrain = super.myDriveTrain;


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
    }


    /**
     * sets our position to the starting position
     */
    @Override
    public void init_loop() {
        super.init_loop();
        //set position
       // if(everythingInit) {
        //    setPose(startingPose);
      //  }
        telemetry.addLine("auto inited");
    }


    @Override
    public void start() {
        super.start();
        // set position again
        stageFinished = true;
        currStage = 0;
        currTimeMillis = SystemClock.uptimeMillis();
    }

    @Override
    public void loop() {
        super.loop();
     //   MainStateMachine();
    }

    // override this
  //  public abstract void MainStateMachine();

}