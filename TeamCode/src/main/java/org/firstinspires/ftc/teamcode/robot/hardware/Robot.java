package org.firstinspires.ftc.teamcode.robot.hardware;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.localization.BaseOdometry;
import org.firstinspires.ftc.teamcode.control.localization.EulerIntegration;
import org.firstinspires.ftc.teamcode.util.Point;
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
    public Pose movementPowers;

    private FtcDashboard dashboard;
    public TelemetryPacket packet;

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

    public Robot(Pose startPose, HardwareMap hardwareMap, FtcDashboard dashboard) {
        frontLeft = hardwareMap.get(ExpansionHubMotor.class, "frontLeft");
        frontRight = hardwareMap.get(ExpansionHubMotor.class, "frontRight");
        backLeft = hardwareMap.get(ExpansionHubMotor.class, "backLeft");
        backRight = hardwareMap.get(ExpansionHubMotor.class, "backRight");
        driveTrain = new DriveTrain(frontLeft, frontRight, backLeft, backRight);

        odometry = new EulerIntegration(startPose);
        currPosition = startPose;
        currVelocity = new Pose();
        movementPowers = new Pose();

        driveHub = hardwareMap.get(ExpansionHubEx.class, "driveHub");
        otherHub = hardwareMap.get(ExpansionHubEx.class, "otherHub");
        driveBulkData = null;

        allHardware = new ArrayList<>();
        allHardware.add(driveTrain);

        this.dashboard = dashboard;
        packet = null;
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
    }

    public void updateDashboard(ArrayList<Point> points) {

    }

}
