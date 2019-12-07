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
             telemetry.addData("scaled world heading", Math.toDegrees(scaledWorldHeadingRad));
             pointAngle(Math.toRadians(45),1.0,Math.toRadians(10));

             if(timedOut(5000)) {
                 nextStage(progStates.stop.ordinal());
             }
         }



         if(currStage == progStates.stop.ordinal()) {
             stopMovement();
             telemetry.addLine("robot is done uwu");
         }
    }
}
