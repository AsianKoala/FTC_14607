package org.firstinspires.ftc.teamcode.robot;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.controllers.Path;
import org.firstinspires.ftc.teamcode.control.controllers.PathPoint;
import org.firstinspires.ftc.teamcode.control.controllers.PurePursuitController;
import org.firstinspires.ftc.teamcode.control.localization.BaseOdometry;
import org.firstinspires.ftc.teamcode.control.localization.EulerIntegration;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

// robot should contain all the data for hardware, including global localization data, bulk reads, debug telem, etc
public class Robot {
    public Pose currPosition;
    public Pose currVelocity;
    public Pose currPoseDelta;
    public Pose currMovementPower;

    private final FtcDashboard dashboard;
    public TelemetryPacket packet;
    public long updateMarker;

    public boolean isFollowing;

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
        currPosition = startPose;
        currVelocity = new Pose();
        currMovementPower = new Pose();

        driveHub = hardwareMap.get(ExpansionHubEx.class, "driveHub");
        otherHub = hardwareMap.get(ExpansionHubEx.class, "otherHub");
        driveBulkData = null;

        allHardware = new ArrayList<>();
        allHardware.add(driveTrain);

        this.dashboard = dashboard;
        packet = null;
        updateMarker = System.nanoTime();

        isFollowing = false;
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

        currPosition = odomPoses[0];
        currVelocity = odomPoses[1];
        currPoseDelta = odomPoses[3];

        for(Hardware h : allHardware)
            h.update(this);

        // dashboard telemetry
        packet = new TelemetryPacket();
        packet.put("pose", currPosition.toString());
        packet.put("movement", currMovementPower.toString());
        packet.put("velocity", currVelocity.toString());

        packet.fieldOverlay()
                .setFill("blue")
                .fillCircle(currPosition.x, currPosition.y, 3);
    }

    public boolean followPath(Path path) {
        return PurePursuitController.followPath(path);
    }

    public void updateDashboard() {
        if(dashboard != null) {
            packet.put("update time", System.nanoTime() - updateMarker);
            dashboard.sendTelemetryPacket(packet);
            updateMarker = System.nanoTime();
        }
    }

    public void updateDashboardPath(Path path) {
        double[] x = new double[path.pathPoints.size()];
        double[] y = new double[path.pathPoints.size()];

        int index = 0;
        for(PathPoint p : path.pathPoints) {
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
