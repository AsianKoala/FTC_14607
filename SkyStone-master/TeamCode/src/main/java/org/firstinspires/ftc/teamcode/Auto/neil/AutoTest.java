package org.firstinspires.ftc.teamcode.Auto.neil;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * ALWAYS EXTEND FROM BASE AUTO
 */
@Autonomous(name = "test new auto", group = "new")
public class AutoTest extends BaseAuto {

    @Override
    public void runOpMode() {
        super.runOpMode(); //! IMPORTANT ! like literally the most important part,
        // everything after this will be ran during the OpMode
        // to add stuff to init remove while(!isStarted) in super class, put it in here, and add whatever u wanted to before that
        //ok







        bestVerticalMovement(48, 1, 0.05, 12, 12, 0.5, Math.toRadians(10), STATUS.firstMovement);

    }


    subMethod turnOnIntake = new subMethod(STATUS.firstMovement) {
        @Override
        protected void method() {
            turnOnIntake();
        }
    };

    subMethod turnOffIntake = new subMethod(STATUS.thirdMovement) {
        @Override
        protected void method() {
            turnOffIntake();
        }
    };




}
