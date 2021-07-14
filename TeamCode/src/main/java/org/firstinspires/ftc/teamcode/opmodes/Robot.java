package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.firstinspires.ftc.teamcode.control.controllers.PurePursuitController;
import org.firstinspires.ftc.teamcode.control.localization.BaseOdometry;
import org.firstinspires.ftc.teamcode.control.localization.EulerIntegration;
import org.firstinspires.ftc.teamcode.control.path.Path;
import org.firstinspires.ftc.teamcode.control.path.PathPoints;
import org.firstinspires.ftc.teamcode.robot.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.Hardware;
import org.firstinspires.ftc.teamcode.util.OpModeClock;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.LinkedList;

@Config
public abstract class Robot extends TunableOpMode {

    public abstract Pose startPose();

    public Pose currPose;
    public Pose currVel;
    public Pose currPoseDelta;
    public Pose currDrivePowers;

    private FtcDashboard dashboard;
    public TelemetryPacket packet;
    public long updateMarker;

    public Path pathCache;
    public LinkedList<PathPoints.BasePathPoint> fullPathCopy;

    private BaseOdometry odometry;
    public RevBulkData driveBulkData;
    public RevBulkData otherBulkData;

    public ExpansionHubEx driveHub;
    public ExpansionHubEx otherHub;

    public ExpansionHubMotor frontLeft;
    public ExpansionHubMotor frontRight;
    public ExpansionHubMotor backLeft;
    public ExpansionHubMotor backRight;
    public DriveTrain driveTrain;

    public ArrayList<Hardware> allHardware;

    public final static int LEFT_ENCODER_PORT = 0;
    public final static int RIGHT_ENCODER_PORT = 1;
    public final static int PERP_ENCODER_PORT = 2;


    @Override
    public void init() {
        OpModeClock.markInit();

        frontLeft = hardwareMap.get(ExpansionHubMotor.class, "frontLeft");
        frontRight = hardwareMap.get(ExpansionHubMotor.class, "frontRight");
        backLeft = hardwareMap.get(ExpansionHubMotor.class, "backLeft");
        backRight = hardwareMap.get(ExpansionHubMotor.class, "backRight");
        driveTrain = new DriveTrain(frontLeft, frontRight, backLeft, backRight);

        odometry = new EulerIntegration(startPose());
        currPose = startPose();
        currVel = new Pose();
        currDrivePowers = new Pose();

        driveHub = hardwareMap.get(ExpansionHubEx.class, "driveHub");
        otherHub = hardwareMap.get(ExpansionHubEx.class, "otherHub");
        driveBulkData = null;

        allHardware = new ArrayList<>();
        allHardware.add(driveTrain);

        pathCache = new Path();

        dashboard = FtcDashboard.getInstance();
        packet = null;
        updateMarker = System.nanoTime();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        OpModeClock.markStart();
    }

    @Override
    public void loop() {
        driveBulkData = driveHub.getBulkInputData();
        otherBulkData = otherHub.getBulkInputData();

        Pose[] odomPoses = odometry.update(new Pose(
                driveBulkData.getMotorCurrentPosition(LEFT_ENCODER_PORT),
                driveBulkData.getMotorCurrentPosition(RIGHT_ENCODER_PORT),
                driveBulkData.getMotorCurrentPosition(PERP_ENCODER_PORT))
        );

        currPose = odomPoses[0];
        currVel = odomPoses[1];
        currPoseDelta = odomPoses[3];

        for(Hardware h : allHardware)
            h.update(this);

        // path cache
        if(pathCache.size() > 1)
            PurePursuitController.followPath(this);

        // dashboard telemetry
        updateTelemetry();
    }


    public void setPathCache(Path path) {
        pathCache = path;
        for (PathPoints.BasePathPoint pathPoint : pathCache) {
            fullPathCopy.add(new PathPoints.BasePathPoint(pathPoint));
        }
    }

    public void updateTelemetry() {
        packet = new TelemetryPacket();
        packet.put("pose", currPose.toString());
        packet.put("movement", currDrivePowers.toString());
        packet.put("velocity", currVel.toString());

        double[] x = new double[fullPathCopy.size()];
        double[] y = new double[fullPathCopy.size()];

        int index = 0;
        for(PathPoints.BasePathPoint p : fullPathCopy) {
            x[index] = p.x;
            y[index] = p.y;
            index++;
        }

        packet.fieldOverlay()
                .setFill("blue")
                .fillCircle(currPose.x, currPose.y, 3)
                .setStroke("red")
                .setStrokeWidth(1)
                .strokePolygon(x, y);

        if(dashboard != null) {
            packet.put("update time", System.nanoTime() - updateMarker);
            dashboard.sendTelemetryPacket(packet);
            updateMarker = System.nanoTime();
        }
    }

}
