package org.firstinspires.ftc.teamcode.lastMinutescuffedPP.Hardware;

import android.os.SystemClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;
import org.firstinspires.ftc.teamcode.lastMinutescuffedPP.util.TelemetryAdvanced;
import org.firstinspires.ftc.teamcode.revextensions2.ExpansionHubEx;
import org.firstinspires.ftc.teamcode.revextensions2.ExpansionHubMotor;
import org.firstinspires.ftc.teamcode.revextensions2.RevBulkData;
import org.firstinspires.ftc.teamcode.revextensions2.RevExtensions2;
import static org.firstinspires.ftc.teamcode.lastMinutescuffedPP.Movement.MyPosition.*;
import static org.firstinspires.ftc.teamcode.lastMinutescuffedPP.Movement.MovementVars.*;
import org.firstinspires.ftc.teamcode.lastMinutescuffedPP.Movement.MyPosition;
import org.firstinspires.ftc.teamcode.lastMinutescuffedPP.Movement.SpeedOmeter;


import java.text.DecimalFormat;
import java.util.ArrayList;

/**
 * base class for opmodes that use the robot ( so like all)
 */
public class Robot extends TunableOpMode {
    // / / / / / C O N S T A N T S / / / / / //
    public final double FIELD_LENGTH = 358.775;

    private RevBulkData revExpansionMasterBulkData;
    private RevBulkData revExpansionSlaveBulkData;
    // expansuion hub objects
    private ExpansionHubEx revMaster;
    private ExpansionHubEx revSlave;
    private BNO055IMU imu;

    private ArrayList<RevMotor> allMotors = new ArrayList<>();

    public Intake myIntake;

    // ppl use this for formatting decimals ?
    public DecimalFormat df = new DecimalFormat("#.00");
    private DriveTrain myDriveTrain;




    public static TelemetryAdvanced m_telemetry;


    // in ms
    public long currTimeMillis = 0;

    /**
     * called when driver pushes init
     */
    public void init() {

        // call this to init hidden features
        RevExtensions2.init();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());


        // get expansion hubs
        revMaster = hardwareMap.get(ExpansionHubEx.class, "Master");
        revSlave = hardwareMap.get(ExpansionHubEx.class, "Slave");



        telemetry.setItemSeparator("");
        telemetry.setMsTransmissionInterval(250);


        // get all drive train motors and map them out, and add them to the all motor array list
        RevMotor tl = new RevMotor((ExpansionHubMotor) hardwareMap.get("TL"),true);
        RevMotor tr = new RevMotor((ExpansionHubMotor) hardwareMap.get("TR"),true);
        RevMotor bl = new RevMotor((ExpansionHubMotor) hardwareMap.get("BL"),true);
        RevMotor br = new RevMotor((ExpansionHubMotor) hardwareMap.get("BR"),true);
        //add them to all motors
        allMotors.add(tl);
        allMotors.add(tr);
        allMotors.add(bl);
        allMotors.add(br);
        //now we can initialize the myDriveTrain
        myDriveTrain = new DriveTrain(tl, tr, bl, br, imu);


        // now we add our intake

        RevMotor leftIntakeMotor = new RevMotor((ExpansionHubMotor) hardwareMap.get("leftIntake"), false);
        RevMotor rightIntakeMotor = new RevMotor((ExpansionHubMotor) hardwareMap.get("rightIntake"), false);
        allMotors.add(leftIntakeMotor);
        allMotors.add(rightIntakeMotor);
        myIntake = new Intake(this, leftIntakeMotor, rightIntakeMotor);



        // TODO:getRevBulkData();

        myIntake.update();
    }

    @Override
    public void init_loop() {
        currTimeMillis = SystemClock.uptimeMillis();
        // TODO: getRevBulkData();

        // check what button press stuff does

        myIntake.update();
    }

    @Override
    public void start() {
        for(int i=0; i<2; i++) {
          //  MyPosition.initialize(myDriveTrain.getCurrentAvgLeft(), myDriveTrain.getCurrentAvgRight()
         //   , myDriveTrain, );
        }
    }

    @Override
    public void loop() {

    }







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






    private long lastUpdateSlaveTime = 0;
    private long lastUpdateMasterTime = 0;

    public void getRevBulkData() {

        RevBulkData newDataMaster;
        try{
            newDataMaster = revMaster.getBulkInputData();
            if(newDataMaster != null){
                revExpansionMasterBulkData = newDataMaster;
            }
        }catch(Exception e){
            //don't set anything if we get an exception
        }
        lastUpdateMasterTime = currTimeMillis;

//        }


        /*
            We don't always need to poll the slave rev hub if we know the collector and lift
            are not moving
         */
        boolean needToPollSlave =   currTimeMillis - lastUpdateSlaveTime > 400;

        if(needToPollSlave){
            RevBulkData newDataSlave;
            try{
                newDataSlave = revSlave.getBulkInputData();
                if(newDataSlave != null){
                    revExpansionSlaveBulkData = newDataSlave;
                }
            }catch(Exception e){
                //don't set anything if we get an exception
            }
            lastUpdateSlaveTime = currTimeMillis;
        }



        /////NOW WE HAVE THE BULK DATA BUT WE NEED TO SET THE MOTOR POSITIONS/////
        for(RevMotor revMotor : allMotors){
            if(revMotor == null){continue;}
            if(revMotor.isMaster){
                if(revExpansionMasterBulkData != null){
                    revMotor.setEncoderReading(
                            revExpansionMasterBulkData.getMotorCurrentPosition(revMotor.myMotor));
                }
            }else{
                if(revExpansionSlaveBulkData != null){
                    revMotor.setEncoderReading(
                            revExpansionSlaveBulkData.getMotorCurrentPosition(revMotor.myMotor));
                }
            }
        }

    }

}
