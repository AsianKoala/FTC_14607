package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses.Auto;

import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.blueFoundationStart;
import static org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.RobotMovement.*;
import static org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.RobotPosition.giveMePose;
import static org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.RobotPosition.scaledWorldHeadingRad;

@Autonomous
public class MovementTest extends Auto {

    private enum progStates {
        openLoopForward,
        forward,
        turn,
        back,
        stop
    }


    @Override
    public void init() {
        super.init();
        giveMePose(blueFoundationStart);
    }


    @Override
    public void init_loop() {
        super.init_loop();
        addSpace();
    }

    @Override
    public void start() {
        super.start();
        giveMePose(blueFoundationStart);
    }

    @Override
    public void loop() {
        super.loop();
    }



    @Override
    public void MainStateMachine() {
         if(currStage == progStates.openLoopForward.ordinal()) {
             if(stageFinished) {
                 initStateVars();
             }

             mecanumPower(0,0.5,0);
             if(timedOut(2000)) {
                 nextStage(progStates.forward.ordinal());
             }
         }


         if(currStage == progStates.forward.ordinal()) {
             if(stageFinished) {
                 initStateVars();
             }

             gunToPosition(stageStartingXPos + 24, stageStartingYPos, 1, 0, 0, 0, 0,true);
            if(timedOut(2000)) {
                nextStage(progStates.stop.ordinal());
            }

         }





         if(currStage == progStates.stop.ordinal()) {
             stopMovement();
             telemetry.addLine("robot is done uwu");
         }
    }
}
