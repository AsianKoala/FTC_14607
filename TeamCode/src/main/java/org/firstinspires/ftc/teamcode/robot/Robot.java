package org.firstinspires.ftc.teamcode.robot;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.path.Path;
import org.firstinspires.ftc.teamcode.control.path.PathPoint;
import org.firstinspires.ftc.teamcode.control.controllers.PurePursuitController;
import org.firstinspires.ftc.teamcode.control.localization.BaseOdometry;
import org.firstinspires.ftc.teamcode.control.localization.EulerIntegration;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.LinkedList;

// robot should contain all the data for hardware, including global localization data, bulk reads, debug telem, etc
public class Robot {
    public Pose currPose;
    public Pose currVel;
    public Pose currPoseDelta;
    public Pose currPowers;

    private final FtcDashboard dashboard;
    public TelemetryPacket packet;
    public long updateMarker;

    public Path pathCache;
    public LinkedList<PathPoint> fullPathCopy;

    private final BaseOdometry odometry;
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

    public Robot(Pose startPose, HardwareMap hardwareMap, FtcDashboard dashboard) {
        frontLeft = hardwareMap.get(ExpansionHubMotor.class, "frontLeft");
        frontRight = hardwareMap.get(ExpansionHubMotor.class, "frontRight");
        backLeft = hardwareMap.get(ExpansionHubMotor.class, "backLeft");
        backRight = hardwareMap.get(ExpansionHubMotor.class, "backRight");
        driveTrain = new DriveTrain(frontLeft, frontRight, backLeft, backRight);

        odometry = new EulerIntegration(startPose);
        currPose = startPose;
        currVel = new Pose();
        currPowers = new Pose();

        driveHub = hardwareMap.get(ExpansionHubEx.class, "driveHub");
        otherHub = hardwareMap.get(ExpansionHubEx.class, "otherHub");
        driveBulkData = null;

        allHardware = new ArrayList<>();
        allHardware.add(driveTrain);

        pathCache = new Path();

        this.dashboard = dashboard;
        packet = null;
        updateMarker = System.nanoTime();
    }

    public void update() {
        driveBulkData = driveHub.getBulkInputData();
        otherBulkData = otherHub.getBulkInputData();

        Pose[] odomPoses = odometry.update(new Pose(
                driveBulkData.getMotorCurrentPosition(LEFT_ENCODER_PORT),
                driveBulkData.getMotorCurrentPosition(RIGHT_ENCODER_PORT),
                driveBulkData.getMotorCurrentPosition(PERP_ENCODER_PORT)),
                new Pose(
                        driveBulkData.getMotorVelocity(LEFT_ENCODER_PORT),
                        driveBulkData.getMotorVelocity(RIGHT_ENCODER_PORT),
                        driveBulkData.getMotorVelocity(PERP_ENCODER_PORT)
        ));

        currPose = odomPoses[0];
        currVel = odomPoses[1];
        currPoseDelta = odomPoses[3];

        for(Hardware h : allHardware)
            h.update(this);

        // path cache
        if(pathCache.size() > 1)
            PurePursuitController.followPath(this);

        // dashboard telemetry
        packet = new TelemetryPacket();
        packet.put("pose", currPose.toString());
        packet.put("movement", currPowers.toString());
        packet.put("velocity", currVel.toString());

        packet.fieldOverlay()
                .setFill("blue")
                .fillCircle(currPose.x, currPose.y, 3);
    }

    public void setPathCache(Path path) {
        pathCache = path;
        for (PathPoint pathPoint : pathCache) {
            fullPathCopy.add(pathPoint.clone());
        }
    }

    public void updateDashboard() {
        if(dashboard != null) {
            packet.put("update time", System.nanoTime() - updateMarker);
            dashboard.sendTelemetryPacket(packet);
            updateMarker = System.nanoTime();
        }
    }

    public void updateDashboardPath() {
        double[] x = new double[fullPathCopy.size()];
        double[] y = new double[fullPathCopy.size()];

        int index = 0;
        for(PathPoint p : fullPathCopy) {
            x[index] = p.x;
            y[index] = p.y;
            index++;
        }

        packet.fieldOverlay()
                .setStroke("red")
                .setStrokeWidth(1)
                .strokePolygon(x, y);
    }

}
