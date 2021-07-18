package org.firstinspires.ftc.teamcode.control.system;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.control.localization.WorkingOdometry;
import org.firstinspires.ftc.teamcode.control.path.Path;
import org.firstinspires.ftc.teamcode.control.path.PathPoints;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.Marker;
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
    public static Pose currVel = new Pose();

    public LinkedList<Path> pathCache;
    public ArrayList<PathPoints.BasePathPoint> fullPathCopy;

    public RevBulkData masterBulkData;
    public RevBulkData slaveBulkData;
    public WorkingOdometry odometry;

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

    @Override
    public void init () {
        Marker.markStart();

        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled  = false;
        imu.initialize(parameters);
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
        headingOffset = imu.getAngularOrientation().firstAngle;

        odometry = new WorkingOdometry(hardwareMap, startPose());

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

        Marker.markEnd("init");
    }

    @Override
    public void init_loop() {
        Marker.markStart();
        updateTelemetry();
        Marker.markEnd("init loop");
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        Marker.markStart();
        updateDataInputComponents();
        Marker.markEnd("data input");

        Marker.markStart();
        updateTelemetry();
        Marker.markEnd("telemetry");

        Marker.markStart();
        updatePath();
        Marker.markEnd("path");

        Marker.markStart();
        updateHardware();
        Marker.markEnd("hardware");

        Marker.markStart();
        updateDashboard();
        Marker.markEnd("dashboard");
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

    private void updateHardware() {
        allHardware.forEach(h -> packet.putAll(h.update()));
    }

    private void updateDataInputComponents() {
        masterBulkData = masterHub.getBulkInputData();
        slaveBulkData = slaveHub.getBulkInputData();
        double lastHeading = imu.getAngularOrientation().firstAngle - headingOffset;
        Pose[] odomData = odometry.realUpdate(MathUtil.angleWrap(lastHeading + startPose().heading));
        currPose = odomData[0];
        currVel = odomData[1];
        currPose = new Pose();
        currVel = new Pose();
    }

    private void updateTelemetry() {
        telemetry.clear();
        packet = new TelemetryPacket();

        packet.put("pose", currPose.toString());
        packet.put("velocity vectors", currVel.toString());

        packet.fieldOverlay()
                .setFill("blue")
                .fillCircle(currPose.x, currPose.y, 3)
                .setStroke("red")
                .setStrokeWidth(1);

        if(!pathCache.isEmpty()) {
            double[] x = new double[fullPathCopy.size()];
            double[] y = new double[fullPathCopy.size()];

            int index = 0;
            for(PathPoints.BasePathPoint p : fullPathCopy) {
                x[index] = p.x;
                y[index] = p.y;
                index++;
            }
            packet.fieldOverlay().strokePolygon(x,y);
        }
    }

    private void updateDashboard() {
        if(dashboard != null) {
            packet.putAll(Marker.telemetryMap);
            packet.put("update time", System.nanoTime() - updateMarker);
            dashboard.sendTelemetryPacket(packet);
            updateMarker = System.nanoTime();
        }
    }
}















