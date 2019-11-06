package org.firstinspires.ftc.teamcode.lastMinutescuffedPP.Hardware;

import android.os.SystemClock;
import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;
import org.firstinspires.ftc.teamcode.lastMinutescuffedPP.util.TelemetryAdvanced;
import org.firstinspires.ftc.teamcode.revextensions2.ExpansionHubEx;
import org.firstinspires.ftc.teamcode.revextensions2.ExpansionHubMotor;
import org.firstinspires.ftc.teamcode.revextensions2.RevBulkData;
import org.firstinspires.ftc.teamcode.revextensions2.RevExtensions2;

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



        // get expansion hubs
        revMaster = hardwareMap.get(ExpansionHubEx.class, "Master");
        revSlave = hardwareMap.get(ExpansionHubEx.class, "Slave");



        telemetry.setItemSeparator("");
        telemetry.setMsTransmissionInterval(250);


        // get all drive train motors and map them out, and add them to the all motor array list
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
    }

}
