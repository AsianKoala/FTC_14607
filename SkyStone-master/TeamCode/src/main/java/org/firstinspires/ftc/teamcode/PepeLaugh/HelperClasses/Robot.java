package org.firstinspires.ftc.teamcode.PepeLaugh.HelperClasses;

import android.os.SystemClock;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import org.openftc.revextensions2.RevBulkData;
import com.qualcomm.robotcore.util.Range;
import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;
import org.firstinspires.ftc.robotcontroller.RobotUtilities.PiecewiseFunction;
import org.firstinspires.ftc.teamcode.PepeLaugh.Debugging.ComputerDebugging;
import org.firstinspires.ftc.teamcode.PepeLaugh.Debugging.TimeProfiler;
import org.firstinspires.ftc.teamcode.PepeLaugh.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.PepeLaugh.Hardware.RevMotor;
import org.firstinspires.ftc.teamcode.PepeLaugh.HelperClasses.Auto;
import org.firstinspires.ftc.teamcode.PepeLaugh.RobotUtilities.ButtonPress;
import org.firstinspires.ftc.teamcode.PepeLaugh.RobotUtilities.MyPosition;
import org.firstinspires.ftc.teamcode.PepeLaugh.RobotUtilities.SpeedOmeter;
import org.firstinspires.ftc.teamcode.PepeLaugh.RobotUtilities.TelemetryAdvanced;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.text.DecimalFormat;
import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.PepeLaugh.RobotUtilities.MovementVars.*;
import static org.firstinspires.ftc.teamcode.PepeLaugh.RobotUtilities.MyPosition.*;

//import teamcode.Auto1;

/**
 * This is the base class for opmodes that use the robot
 */
public class Robot extends TunableOpMode {
    public static boolean usingComputer = true;
    //////////////////////////////////CONSTANTS/////////////////////////
    public final double FIELD_LENGTH = 358.775;//the length of the fieldld in cm
    ////////////////////////////////////////////////////////////////////


    private RevBulkData revExpansionMasterBulkData;
    private RevBulkData revExpansionSlaveBulkData;
    //these are the expansion hub objects
    private ExpansionHubEx revMaster;
    private ExpansionHubEx revSlave;
    //holds all the rev expansion hub motors
    private ArrayList<RevMotor> allMotors = new ArrayList<>();

    ComputerDebugging computerDebugging;

  /*  public Collector myCollector;
    public Lift myLift;
    public AutoFeeder myAutoFeeder;
    public AutoCollector myAutoCollector;
    public HangMechanism hangMechanism; */

    //use this to format decimals lol
    public DecimalFormat df = new DecimalFormat("#.00");
    private DriveTrain myDriveTrain;

    public static TelemetryAdvanced m_telemetry;


    //this will be a milisecond time
    public long currTimeMillis = 0;


    //set this to change the initial position of the collector dumper
    public static double initialCollectorDumperPosition = 0.0;
    /** If we need to fit within the sizing requirement on init. Set this. */
    private boolean startIn18Inches = false;

    /** Sets if we need to start within 18 inches or not */
    public void setStartIn18Inches(boolean startIn18Inches){ this.startIn18Inches = startIn18Inches; }



    /**
     * Called when the driver pushes the init button
     */
    @Override
    public void init() {
        Auto.masterMotorScale = 1.0;
        currTimeMillis = SystemClock.uptimeMillis();
        //we need to call this to initialize the extra features of rev
        RevExtensions2.init();


        //get the two expansion hubs themselves
        revMaster = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 4");
        revSlave = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 2");


        this.msStuckDetectInit = 5000;
        this.msStuckDetectInitLoop = 5000;
        telemetry.setItemSeparator("");
        telemetry.setMsTransmissionInterval(250);
        telemetry.setAutoClear(true);


//        m_telemetry = new TelemetryAdvanced(55,46);//59,53);


        if(usingComputer){
            //use this to debug things
            computerDebugging = new ComputerDebugging();
        }







        ///GET ALL THE DRIVE TRAIN MOTORS AND ADD THEM TO ALL MOTORS///
        RevMotor tl = new RevMotor((ExpansionHubMotor) hardwareMap.get("motorTL"),true);
        RevMotor tr = new RevMotor((ExpansionHubMotor) hardwareMap.get("motorTR"),true);
        RevMotor bl = new RevMotor((ExpansionHubMotor) hardwareMap.get("motorBL"),true);
        RevMotor br = new RevMotor((ExpansionHubMotor) hardwareMap.get("motorBR"),true);
        //add them to all motors
        allMotors.add(tl);
        allMotors.add(tr);
        allMotors.add(bl);
        allMotors.add(br);
        //now we can initialize the myDriveTrain
        myDriveTrain = new DriveTrain(tl, tr, bl, br);





        //get the bulk data if you want to calibrate stuff

    }

    @Override
    public void init_loop(){
//        giveCppFieldData();
        currTimeMillis = SystemClock.uptimeMillis();


        giveButtonPressStuff();
        currTimeMillis = SystemClock.uptimeMillis();




        //draw the robot on the computer if we are usingComputer

    }


    @Override
    public void start() {
        //give MyPosition our current positions so that it saves the last positions of the wheels
        //this means we won't teleport when we start the match. Just in case, run this twice
        for(int i = 0; i < 2 ; i ++){
            MyPosition.initialize(myDriveTrain.topRight.getCurrentPosition(),
                    myDriveTrain.topLeft.getCurrentPosition(),
                    myDriveTrain.bottomLeft.getCurrentPosition(), this);
        }
    }


    private TimeProfiler tp1 = new TimeProfiler(1000);
    private TimeProfiler tp2 = new TimeProfiler(1000);
    private TimeProfiler tp3 = new TimeProfiler(1000);
    private TimeProfiler tp4 = new TimeProfiler(1000);
    private TimeProfiler tp5 = new TimeProfiler(1000);
    private TimeProfiler tp6 = new TimeProfiler(1000);
    private TimeProfiler tp7 = new TimeProfiler(1000);
    private TimeProfiler tp8 = new TimeProfiler(1000);


    /**
     * The time of the last loop update in millis
     */
    private long lastLoopTime = 0;
    /**
     * Amount of time elapsed this update in millis
     */
    public int elapsedMillisThisUpdate = 0;



    private TimeProfiler timeProfiler = new TimeProfiler(300);
    public static double updatesPerSecond = 1000;

    /**
     * Called every update
     */
    @Override
    public void loop() {
        timeProfiler.markEnd();
        timeProfiler.markStart();

        updatesPerSecond = 1000.0/timeProfiler.getAverageTimePerUpdateMillis();
        telemetry.addLine("UPS: " + updatesPerSecond);




        long timeBefore = SystemClock.uptimeMillis();
        telemetry.addLine("Num hardware writes: " + RevMotor.numHardwareUsesThisUpdate);
        RevMotor.markEndUpdate();//mark the end of an update for the rev motor class
        tp1.markStart();
        //get all the bulk data
        tp1.markEnd();

        long timeAfter = SystemClock.uptimeMillis();
        telemetry.addData("Bulk data time: ", (timeAfter-timeBefore));

        //update the computer if we are debugging





        //uses opmode tuner to tune servo positions if enabled
//        tuneServoPositionsWithTuner();
//        tuneScalingFactorsWithTuner();
//        tuneRandomThings();
        //get the current time for everyone
        currTimeMillis = SystemClock.uptimeMillis();
        elapsedMillisThisUpdate = (int) (currTimeMillis - lastLoopTime);
        lastLoopTime = currTimeMillis;

        //update myAutoFeeder
     //   myAutoFeeder.update();
        //update autoCollector
       // myAutoCollector.update();
        tp2.markStart();

        //apply the movement to the drivetrain
        myDriveTrain.ApplyMovement();

        tp2.markEnd();
        tp3.markStart();
        //update the collector
    //    myCollector.update();
        tp3.markEnd();

        tp4.markStart();
        //update the lift
     //   myLift.update();
        tp4.markEnd();
        //update our hangMechanism
     //   hangMechanism.update();

        tp5.markStart();
        //finally, update the PitScannerInterface
//        myPitScannerInterface.update();
        tp5.markEnd();


        tp6.markStart();
        tp6.markEnd();

        //display the readings of the loading sensors for debugging
//        telemetry.addLine("\nLEFT LOADING: " + myLoadingSensors.leftSensorCurrentReading);
//        telemetry.addLine("RIGHT LOADING: " + myLoadingSensors.rightSensorCurrentReading + "\n");



        tp7.markStart();
        //Figures out when all the buttons are pressed for ease of use in other places
        giveButtonPressStuff();

        //this will call MyPosition's update and calculate our position
        MyPosition.giveMePositions(
                myDriveTrain.topRight.getCurrentPosition(),
                myDriveTrain.topLeft.getCurrentPosition(),
                myDriveTrain.bottomLeft.getCurrentPosition());

        tp7.markEnd();

        tp8.markStart();
        //calculate our current speed in every dimension
        SpeedOmeter.update();


        //tell the FtcRobotControllerVisionActivity the robot position
        tp8.markEnd();





//        telemetry.addLine("profile 1: " + tp1.getAverageTimePerUpdateMillis());
//        telemetry.addLine("profile 2: " + tp2.getAverageTimePerUpdateMillis());
//        telemetry.addLine("profile 3: " + tp3.getAverageTimePerUpdateMillis());
//        telemetry.addLine("profile 4: " + tp4.getAverageTimePerUpdateMillis());
//        telemetry.addLine("profile 5: " + tp5.getAverageTimePerUpdateMillis());
//        telemetry.addLine("profile 6: " + tp6.getAverageTimePerUpdateMillis());
//        telemetry.addLine("profile 7: " + tp7.getAverageTimePerUpdateMillis());
//        telemetry.addLine("profile 8: " + tp8.getAverageTimePerUpdateMillis());
    }

    /**
     * Sees if we need to update the loading sensors and updates them if so
     */

    public void stop(){

    }


    private void giveButtonPressStuff() {
        ButtonPress.giveMeInputs(gamepad1.a,gamepad1.b,gamepad1.x,gamepad1.y,gamepad1.dpad_up,
                gamepad1.dpad_down,gamepad1.dpad_right,gamepad1.dpad_left,gamepad1.right_bumper,
                gamepad1.left_bumper,gamepad1.left_stick_button,gamepad1.right_stick_button,
                gamepad2.a,gamepad2.b,gamepad2.x,gamepad2.y,gamepad2.dpad_up,
                gamepad2.dpad_down,gamepad2.dpad_right,gamepad2.dpad_left,gamepad2.right_bumper,
                gamepad2.left_bumper,gamepad2.left_stick_button,gamepad2.right_stick_button);
    }


    /**
     * Kills all movement
     */
    public void stopMovement(){
        movement_x = 0;
        movement_y = 0;
        movement_turn = 0;
    }

    public static String movementYVisual =
                    "                   1" +
                    "                    " +
                    "                    " +
                    "                    " +
                    "                    " +
                    "                    " +
                    "                    " +
                    "                    " +
                    "1                   ";
    public static PiecewiseFunction movementYPiecewise = new PiecewiseFunction(movementYVisual);

    public static String movementXVisual =
                    "                   1" +
                    "                    " +
                    "                    " +
                    "                    " +
                    "                    " +
                    "                    " +
                    "                    " +
                    "                    " +
                    "1                   ";
    public static PiecewiseFunction movementXPiecewise = new PiecewiseFunction(movementXVisual);

    public static String movementTurnVisual =
                    "                   1" +
                    "                    " +
                    "                    " +
                    "                    " +
                    "                    " +
                    "                    " +
                    "                    " +
                    "                    " +
                    "1                   ";
    public static PiecewiseFunction movementTurnPiecewise = new PiecewiseFunction(movementTurnVisual);


    /**
     * Use this to tune the telemetry scaling factors with the FTC opmode tuner
     */
    public void tuneScalingFactorsWithTuner(){
        MyPosition.moveScalingFactor = getDouble("MoveScale");
        MyPosition.turnScalingFactor = getDouble("TurnScale");
        MyPosition.auxScalingFactor = getDouble("AuxScale");
        MyPosition.auxPredictionScalingFactor = getDouble("AuxPrediction");
    }

    /**
     * Accepts user input to control robot
     */
    public void ControlMovement() {
        //go faster with right bumper
        double masterScale = 0.5 + ((gamepad1.right_bumper ? 1 : 0) * (1.0-0.5));
        movement_y = -gamepad1.right_stick_y * masterScale;// * getDouble("y_move_scale");
        movement_x = gamepad1.right_stick_x * masterScale;// * getDouble("x_move_scale");
        movement_turn = -((gamepad1.left_stick_x * masterScale) + (gamepad2.right_stick_x * 0.5)) ;// * getDouble ("rot_move_scale");

        movement_y = (movement_y >= 0 ? 1.0 : -1.0) * movementYPiecewise.getVal(Math.abs(movement_y));
        movement_x = (movement_x >= 0 ? 1.0 : -1.0) * movementXPiecewise.getVal(Math.abs(movement_x));
        movement_turn = (movement_turn >= 0 ? 1.0 : -1.0) * movementTurnPiecewise.getVal(Math.abs(movement_turn));
    }


    /**
     *
     * TODO:
     * VERY IMPORTANT *****
     * GLUTEN FREE HAS A REALLY GOOD CODE SAMPLE ON FOLLOWING POINT CURVES, SUCH AS THE CURVE THAT GOT THEM TO THE FEEDING POSITION FOR AUTO LAST YEAR
     * IF YOU NEED HELP PLEASE LOOK AT THAT
     * ***
     * @return
     */



    public double getXPos(){
        return worldXPosition;
    }
    public double getYPos(){
        return worldYPosition;
    }
    public double getAngle_rad(){
        return worldAngle_rad;
    }
    public double getAngle_deg(){
        return Math.toDegrees(worldAngle_rad);
    }



}

