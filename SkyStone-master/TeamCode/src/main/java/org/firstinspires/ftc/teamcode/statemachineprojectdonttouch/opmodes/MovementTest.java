package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses.Auto;

import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.*;
import static org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.RobotMovement.*;
import static org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.RobotPosition.*;

@Autonomous
public class MovementTest extends Auto {
    public boolean completedMovement(double targetX, double targetY) {
        return Math.hypot(targetX - scaledWorldXPos, targetY - scaledWorldYPos) < 2;
    }

    enum progStates {
        firstMovement,
        fourthMovement,
        fifthMovement,
        sixthMovement,
        seventhMovement,
        eighthMovement,
        ninthMovement,
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
    }

    @Override
    public void loop() {
        super.loop();
    }


    @Override
    public void MainStateMachine() {
        if(currStage == progStates.firstMovement.ordinal()) {
            if(stageFinished) {
                initStateVars();
                scaledWorldXPos = 0;
                scaledWorldYPos = 0;
                scaledWorldHeadingRad = 0;
            }

            gunToPosition(20,0,1,0,0.5,0,0,true);

            if(completedMovement(20,0)) {
                nextStage(progStates.stop.ordinal());
            }
        }

        if(currStage == progStates.stop.ordinal()) {
            stopMovement();
        }
    }
}
