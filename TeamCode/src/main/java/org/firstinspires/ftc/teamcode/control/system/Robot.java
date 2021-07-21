package org.firstinspires.ftc.teamcode.control.system;

import android.annotation.SuppressLint;
import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.control.localization.Odometry;
import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.control.path.Path;
import org.firstinspires.ftc.teamcode.control.path.PathPoints;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.Debuggable;
import org.firstinspires.ftc.teamcode.util.Mar;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.OpModeType;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

import static org.firstinspires.ftc.teamcode.util.MathUtil.sgn;

@Config
public abstract class Robot extends TunableOpMode {

    public abstract Pose startPose();

    public static Pose currPose = new Pose();
//    public static Pose currVel = new Pose();

    public Path pathCache;

    public RevBulkData masterBulkData;
    public RevBulkData slaveBulkData;
    public Odometry odometry;
    private BNO055IMU imu;
    private double headingOffset;

    private FtcDashboard dashboard;
    public TelemetryPacket packet;

    public ExpansionHubEx masterHub;
    public ExpansionHubEx slaveHub;

    public ExpansionHubMotor frontLeft;
    public ExpansionHubMotor frontRight;
    public ExpansionHubMotor backLeft;
    public ExpansionHubMotor backRight;

    public DriveTrain driveTrain;
    public ArrayList<Hardware> allHardware;

    private Mar initTime;
    private Mar init_loopTime;
    private Mar loopTime;
    private Mar hwTime;
    private Mar telemTime;
    private Mar pathTime;
    private Mar dashTime;
    private double maxLT;

    private double lastManualUpdate;
    private double lastAutoUpdate;
    private boolean isDebugging;
    private boolean isPathStopped;
    private OpModeType type;
    private Pose debugSpeeds;

    // optimization shit goes here @TODO
    // (not supposed to be a joke)

    @Override
    public void init () {
        initTime = new Mar();
        initTime.start();
        init_loopTime = new Mar();
        loopTime = new Mar();
        hwTime = new Mar();
        telemTime = new Mar();
        pathTime = new Mar();
        dashTime = new Mar();
        maxLT = 0;

        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled  = false;
        imu.initialize(parameters);
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
        headingOffset = imu.getAngularOrientation().firstAngle;

        odometry = new Odometry(startPose());

        masterHub = hardwareMap.get(ExpansionHubEx.class, "masterHub");
        slaveHub = hardwareMap.get(ExpansionHubEx.class, "slaveHub");

        frontLeft = hardwareMap.get(ExpansionHubMotor.class, "FL");
        frontRight = hardwareMap.get(ExpansionHubMotor.class, "FR");
        backLeft = hardwareMap.get(ExpansionHubMotor.class, "BL");
        backRight = hardwareMap.get(ExpansionHubMotor.class, "BR");

        driveTrain = new DriveTrain(frontLeft, frontRight, backLeft, backRight);

        allHardware = new ArrayList<>();
        allHardware.add(driveTrain);

        pathCache = new Path();

        dashboard = FtcDashboard.getInstance();
        packet = null;
        initTime.stop();

        lastManualUpdate = System.currentTimeMillis();
        lastAutoUpdate = System.currentTimeMillis();
        isDebugging = getClass().isAnnotationPresent(Debuggable.class);
        isPathStopped = true;
        type = getClass().isAnnotationPresent(Autonomous.class) ? OpModeType.AUTO : OpModeType.TELEOP;
        debugSpeeds = new Pose();
    }

    @Override
    public void init_loop() {
        init_loopTime.start();
        updateTelemetry();
        init_loopTime.stop();
    }

    @Override
    public void start() {
        telemetry.clear();
    }

    @Override
    public void loop() {
        loopTime.start();
        hwTime.start();
        if(isDebugging) debugControl();
        else updateHardware();
        hwTime.stop();
        telemTime.start();
        updateTelemetry();
        telemTime.stop();
        pathTime.start();
        updatePath();
        pathTime.stop();
        dashTime.start();
        updateDashboard();
        dashTime.stop();
        loopTime.stop();
    }

    protected void setPathCache(Path path) {
        pathCache = path;
    }

    private void updatePath() {
        if(!pathCache.finished()) {
            pathCache.follow(this);
        } else {
            pathCache.remove(0);
            if(type == OpModeType.AUTO && !isDebugging)
                DriveTrain.powers.set(new Pose());
        }
    }

    protected void updateHardware() {
        masterBulkData = masterHub.getBulkInputData();
        slaveBulkData = slaveHub.getBulkInputData();
        double lastHeading = imu.getAngularOrientation().firstAngle - headingOffset;
        currPose.set(odometry.update(slaveBulkData.getMotorCurrentPosition(2),
                slaveBulkData.getMotorCurrentPosition(0),
                MathUtil.angleWrap(lastHeading + startPose().h)));
    }

    private void updateTelemetry() {
        packet = new TelemetryPacket();
        packet.addLine("FTC 14607 by Neil Mehra");
        Date buildDate = new Date(BuildConfig.TIMESTAMP);
        @SuppressLint("SimpleDateFormat") SimpleDateFormat dateFormat = new SimpleDateFormat("MM/dd HH:mm:ss");
        packet.put("built at", dateFormat.format(buildDate));

        packet.put("init time", initTime.getTime());
        packet.put("init loop time", init_loopTime.getTime());
        packet.put("loop time", loopTime.getTime());
        if(loopTime.getTime() > maxLT) {
            maxLT = loopTime.getTime();
        }
        packet.put("max loop time", maxLT);
        packet.put("hw time", hwTime.getTime());
        packet.put("telem time", telemTime.getTime());
        packet.put("path time", pathTime.getTime());
        packet.put("dash time", dashTime.getTime());

        packet.put("ftc sdk ver", BuildConfig.VERSION_NAME);
        packet.put("java ver", System.getProperty("java.runtime.version"));
        packet.addLine(Build.MANUFACTURER + " " + Build.MODEL + " running android sdk " + Build.VERSION.SDK_INT);

        packet.put("hub ver", masterHub.getFirmwareVersion());
        List<LynxModule> revHubs = hardwareMap.getAll(LynxModule.class);
        List<DcMotor> motors = hardwareMap.getAll(DcMotor.class);
        List<Servo> servos = hardwareMap.getAll(Servo.class);
        List<DigitalChannel> digital = hardwareMap.getAll(DigitalChannel.class);
        List<AnalogInput> analog = hardwareMap.getAll(AnalogInput.class);
        List<I2cDevice> i2c = hardwareMap.getAll(I2cDevice.class);
        packet.addLine(revHubs.size() + " Hubs; " + motors.size() + " Motors; " + servos.size() +
                " Servos; " + (digital.size() + analog.size() + i2c.size()) + " Sensors");

        packet.put("x", currPose.x);
        packet.put("y", currPose.y);
        packet.put("h", Math.toDegrees(currPose.h));
        packet.put("odometry", odometry.toString());
        packet.put("vector powers", DriveTrain.powers.toString());
        allHardware.forEach(h -> packet.putAll(h.update()));

        if(pathCache.isEmpty()) {
            packet.addLine("path empty");
        } else {
            packet.put("point amts", pathCache.size());
            packet.put("current path name", pathCache.name);
            packet.put("current target name", pathCache.get(0).signature);
        }
        packet.fieldOverlay()
                .setFill("blue")
                .fillCircle(currPose.y-72, -(currPose.x-72), 3);
    }

    private void updateDashboard() {
        if(dashboard != null)
            dashboard.sendTelemetryPacket(packet);
    }

    private void debugControl() {
        if(gamepad1.a) isPathStopped = !isPathStopped;
        if (System.currentTimeMillis() - lastManualUpdate > 500) {
            currPose.set(new Pose(
                    currPose.x + sgn(gamepad1.left_stick_x),
                    currPose.y - sgn(gamepad1.left_stick_y),
                    currPose.h + (sgn(gamepad1.right_stick_x) * (Math.PI / 10))
            ));
            lastManualUpdate = System.currentTimeMillis();
        }
        if(isDebugging) {
            double elapsed = (System.currentTimeMillis() - lastAutoUpdate)/1000.0;
            lastAutoUpdate = System.currentTimeMillis();
            if(elapsed > 1) return;

            double radius = debugSpeeds.hypot();
            double theta = currPose.h + debugSpeeds.atan() - Math.PI / 2;
            currPose.set(currPose.add(new Pose(
                    radius * Math.cos(theta) * elapsed * 1000 * 0.2,
                    radius * Math.sin(theta) * elapsed * 1000 * 0.2,
                    DriveTrain.powers.h * elapsed * 20 / (2 * Math.PI))));

            debugSpeeds.set(debugSpeeds.add(new Pose(
                    MathUtil.clip((DriveTrain.powers.x - debugSpeeds.x)/0.2,-1,1) * elapsed,
                    MathUtil.clip((DriveTrain.powers.y - debugSpeeds.y)/0.2,-1,1) * elapsed,
                    MathUtil.clip((DriveTrain.powers.h - debugSpeeds.h)/0.2,-1,1) * elapsed
            )).multiply(new Pose(1.0 - elapsed)));
        }
    }
}















