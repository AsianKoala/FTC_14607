package org.firstinspires.ftc.teamcode.control.system;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.control.localization.Odometry;
import org.firstinspires.ftc.teamcode.control.localization.OdometrySet;
import org.firstinspires.ftc.teamcode.control.path.Path;
import org.firstinspires.ftc.teamcode.control.path.PathPoints;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.Mar;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.LinkedList;

@Config
public abstract class Robot extends TunableOpMode {

    public abstract Pose startPose();

    public static Pose currPose = new Pose();
//    public static Pose currVel = new Pose();

    public LinkedList<Path> pathCache;
    public ArrayList<PathPoints.BasePathPoint> fullPathCopy;

    public RevBulkData masterBulkData;
    public RevBulkData slaveBulkData;
    public Odometry odometry;
    public OdometrySet odometrySet;


    private FtcDashboard dashboard;
    public TelemetryPacket packet;
    public long updateMarker;

    public ExpansionHubEx masterHub;
    public ExpansionHubEx slaveHub;

    public ExpansionHubMotor frontLeft;
    public ExpansionHubMotor frontRight;
    public ExpansionHubMotor backLeft;
    public ExpansionHubMotor backRight;

    public DriveTrain driveTrain;

    public ArrayList<Hardware> allHardware;

    private BNO055IMU imu;
    private double headingOffset;

    private Mar initTime;
    private Mar init_loopTime;
    private Mar loopTime;

    @Override
    public void init () {
        initTime = new Mar();
        initTime.start();
        init_loopTime = new Mar();
        loopTime = new Mar();

        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled  = false;
        imu.initialize(parameters);
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
        headingOffset = imu.getAngularOrientation().firstAngle;

        ExpansionHubMotor verticalOdometer = hardwareMap.get(ExpansionHubMotor.class, "leftIntake");
        ExpansionHubMotor horizontalOdometer = hardwareMap.get(ExpansionHubMotor.class, "rightIntake");
        odometrySet = new OdometrySet(verticalOdometer, horizontalOdometer);
        odometry = new Odometry(startPose(), odometrySet);

        masterHub = hardwareMap.get(ExpansionHubEx.class, "masterHub");
        slaveHub = hardwareMap.get(ExpansionHubEx.class, "slaveHub");

        frontLeft = hardwareMap.get(ExpansionHubMotor.class, "FL");
        frontRight = hardwareMap.get(ExpansionHubMotor.class, "FR");
        backLeft = hardwareMap.get(ExpansionHubMotor.class, "BL");
        backRight = hardwareMap.get(ExpansionHubMotor.class, "BR");

        driveTrain = new DriveTrain(frontLeft, frontRight, backLeft, backRight);

        allHardware = new ArrayList<>();
        allHardware.add(driveTrain);

        pathCache = new LinkedList<>();
        fullPathCopy = new ArrayList<>();

        dashboard = FtcDashboard.getInstance();
        packet = null;
        updateMarker = System.currentTimeMillis();
        initTime.end();
    }

    @Override
    public void init_loop() {
        init_loopTime.start();
        updateTelemetry();
        init_loopTime.end();
    }

    @Override
    public void start() {
        telemetry.clear();
    }

    @Override
    public void loop() {
        loopTime.start();
        updateDataInputComponents();
        updateTelemetry();
        updatePath();
        updateDashboard();
        loopTime.end();
    }

    public void setPathCache(LinkedList<Path> pathList) {
        pathCache.addAll(pathList);
        for(Path e : pathList)
            fullPathCopy.addAll(e);
    }

    public void setDirectPath(Path e) {
        pathCache.add(e);
        fullPathCopy.addAll(e);
    }

    private void updatePath() {
        if(pathCache.size() != 0) {
            pathCache.getFirst().follow(this);
            if(pathCache.getFirst().finished()) {
                pathCache.removeFirst();
                DriveTrain.powers.set(new Pose());
            }
        }
    }

    private void updateDataInputComponents() {
        masterBulkData = masterHub.getBulkInputData();
        slaveBulkData = slaveHub.getBulkInputData();
        double lastHeading = imu.getAngularOrientation().firstAngle - headingOffset;
        odometry.update(MathUtil.angleWrap(lastHeading + startPose().heading));
        currPose = Odometry.currentPosition;

    }

    private void updateTelemetry() {
        packet = new TelemetryPacket();

        packet.put("x", currPose.x);
        packet.put("y", currPose.y);
        packet.put("h", Math.toDegrees(currPose.heading));
        packet.put("odometry", odometry.toString());
        packet.put("vector powers", DriveTrain.powers.toString());
        allHardware.forEach(h -> packet.putAll(h.update()));
        packet.put("init time", initTime.getTime());
        packet.put("init loop time", init_loopTime.getTime());
        packet.put("loop time", loopTime.getTime());
        packet.put("path amts", pathCache.size());
        packet.put("point amts", pathCache.getFirst().size());
        packet.put("current path name", pathCache.getFirst().name);
        packet.put("current target name", pathCache.getFirst().getFirst().signature);
        packet.fieldOverlay()
                .setFill("blue")
                .fillCircle(currPose.y-72, -(currPose.x-72), 3);

        if(!pathCache.isEmpty()) {
            double[] x = new double[fullPathCopy.size()];
            double[] y = new double[fullPathCopy.size()];

            int index = 0;
            for(PathPoints.BasePathPoint p : fullPathCopy) {
                x[index] = p.x;
                y[index] = p.y;
                index++;
            }
            packet.fieldOverlay()
                    .setStroke("red")
                    .setStrokeWidth(1)
                    .strokePolygon(x,y);
        }
    }

    private void updateDashboard() {
        if(dashboard != null) {
            packet.put("update time", System.nanoTime() - updateMarker);
            dashboard.sendTelemetryPacket(packet);
            updateMarker = System.nanoTime();
        }
    }
}















