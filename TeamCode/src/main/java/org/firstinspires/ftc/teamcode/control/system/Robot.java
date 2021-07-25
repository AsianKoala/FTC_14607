package org.firstinspires.ftc.teamcode.control.system;

import android.annotation.SuppressLint;
import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.control.localization.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.control.path.Path;
import org.firstinspires.ftc.teamcode.control.path.PathPoints;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.DataPacket;
import org.firstinspires.ftc.teamcode.util.Debuggable;
import org.firstinspires.ftc.teamcode.util.Mar;
import org.firstinspires.ftc.teamcode.util.OpModeType;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

import static org.firstinspires.ftc.teamcode.util.MathUtil.angleWrap;
import static org.firstinspires.ftc.teamcode.util.MathUtil.sgn;

@Config
public abstract class Robot extends TunableOpMode {

    public abstract Pose startPose();
    public abstract Path path();

    public Pose currPose;
    public Pose currVel;

    public Path pathCache;

    public ExpansionHubEx masterHub;
    public ExpansionHubEx slaveHub;

    public RevBulkData masterBulkData;
    public RevBulkData slaveBulkData;

    private BNO055IMU imu;
    private double headingOffset;
    public TwoWheelTrackingLocalizer odometry;

    public static int PARALLEL_ENCODER_PORT = 2;
    public static int PERP_ENCODER_PORT = 0;

    public ExpansionHubMotor frontLeft;
    public ExpansionHubMotor frontRight;
    public ExpansionHubMotor backLeft;
    public ExpansionHubMotor backRight;

    public DriveTrain driveTrain;
    public ArrayList<Hardware> allHardware;

    private FtcDashboard dashboard;
    public DataPacket packet;

    private Mar initTime;
    private Mar init_loopTime;
    private Mar loopTime;
    private Mar odoTime;
    private Mar telemTime;
    private Mar pathTime;
    private Mar dashTime;
    private double maxLT;

    private double lastManualUpdate;
    private double lastAutoUpdate;
    private boolean debugging;
    private boolean pathStopped;
    private OpModeType type;
    private Pose debugSpeeds;

    // optimization shit goes here TODO
    // (not supposed to be a joke)

    @Override
    public void init () {
        initTime = new Mar();
        initTime.start();
        init_loopTime = new Mar();
        loopTime = new Mar();
        odoTime = new Mar();
        telemTime = new Mar();
        pathTime = new Mar();
        dashTime = new Mar();
        maxLT = -1;

        masterHub = hardwareMap.get(ExpansionHubEx.class, "masterHub");
        slaveHub = hardwareMap.get(ExpansionHubEx.class, "slaveHub");
        masterBulkData = masterHub.getBulkInputData();
        slaveBulkData = slaveHub.getBulkInputData();

        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled  = false;
        imu.initialize(parameters);
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
        headingOffset = imu.getAngularOrientation().firstAngle;

        odometry = new TwoWheelTrackingLocalizer(startPose());
        currPose = startPose();
        currVel = new Pose();

        frontLeft = hardwareMap.get(ExpansionHubMotor.class, "FL");
        frontRight = hardwareMap.get(ExpansionHubMotor.class, "FR");
        backLeft = hardwareMap.get(ExpansionHubMotor.class, "BL");
        backRight = hardwareMap.get(ExpansionHubMotor.class, "BR");

        driveTrain = new DriveTrain(frontLeft, frontRight, backLeft, backRight);

        allHardware = new ArrayList<>();
        allHardware.add(driveTrain);

        setPathCache(path());

        lastManualUpdate = System.currentTimeMillis();
        lastAutoUpdate = System.currentTimeMillis();
        debugging = getClass().isAnnotationPresent(Debuggable.class);
        pathStopped = true;
        type = getClass().isAnnotationPresent(Autonomous.class) ? OpModeType.AUTO : OpModeType.TELEOP;
        debugSpeeds = new Pose();

        dashboard = FtcDashboard.getInstance();
        initTime.stop();
        updateTelemetry(true);
    }

    @Override
    public void init_loop() {
        init_loopTime.start();
        updateTelemetry(true);
        init_loopTime.stop();
    }

    @Override
    public void start() {
        telemetry.clear();
    }

    @Override
    public void loop() {
        loopTime.start();
        odoTime.start();
        if(debugging) debugControl();
        else updateOdo();
        odoTime.stop();
        telemTime.start();
        updateTelemetry(false);
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
        if(path == null) {
            pathCache = new Path();
        } else {
            pathCache = path;
        }
    }

    private void updatePath() {
        if(!pathCache.isEmpty()) {
            pathCache.follow(this);
        }
        if(pathCache.finished() && type==OpModeType.AUTO) {
            driveTrain.powers = new Pose(); // change this ??
        }
    }

    private void updateOdo() {
        masterBulkData = masterHub.getBulkInputData();
        slaveBulkData = slaveHub.getBulkInputData();
        for(int i=0; i<3; i++) {
            try {
                packet.addData(i + " current position", slaveBulkData.getMotorCurrentPosition(i));
            } catch (Exception ignored) {

            }
        }
        double lastHeading = imu.getAngularOrientation().firstAngle - headingOffset;
        odometry.update(this, slaveBulkData, lastHeading);
        currPose = odometry.getPoseUpdate();
    }

    private void updateTelemetry(boolean isInit) {
        packet = new DataPacket();
        packet.addLine(" FTC 14607 by Neil Mehra");
        if(isInit) {
            Date buildDate = new Date(BuildConfig.TIMESTAMP);
            @SuppressLint("SimpleDateFormat") SimpleDateFormat dateFormat = new SimpleDateFormat("MM/dd HH:mm:ss");
            packet.addData("built at", dateFormat.format(buildDate));
            packet.addData("ftc sdk ver", BuildConfig.VERSION_NAME);
            packet.addLine(Build.MANUFACTURER + " " + Build.MODEL + " running android sdk " + Build.VERSION.SDK_INT);

            packet.addData("hub ver", masterHub.getFirmwareVersion());
            List<LynxModule> revHubs = hardwareMap.getAll(LynxModule.class);
            List<DcMotor> motors = hardwareMap.getAll(DcMotor.class);
            List<Servo> servos = hardwareMap.getAll(Servo.class);
            List<DigitalChannel> digital = hardwareMap.getAll(DigitalChannel.class);
            List<AnalogInput> analog = hardwareMap.getAll(AnalogInput.class);
            List<I2cDevice> i2c = hardwareMap.getAll(I2cDevice.class);
            packet.addLine(revHubs.size() + " Hubs; " + motors.size() + " Motors; " + servos.size() +
                    " Servos; " + (digital.size() + analog.size() + i2c.size()) + " Sensors");
            packet.addSpace();
        }

        packet.addData("init time", initTime.getTime());
        packet.addData("init loop time", init_loopTime.getTime());
        packet.addData("loop time", loopTime.getTime());
        if(loopTime.getTime() > maxLT) {
            maxLT = loopTime.getTime();
        }
        packet.addData("max loop time", maxLT);
        packet.addData("hw time", odoTime.getTime());
        packet.addData("telem time", telemTime.getTime());
        packet.addData("path time", pathTime.getTime());
        packet.addData("dash time", dashTime.getTime());

        packet.addSpace();
        packet.addData("debugging", debugging);
        packet.addData("debug path stopped", pathStopped);

        packet.addSpace();
        packet.addData("x", currPose.x);
        packet.addData("y", currPose.y);
        packet.addData("h", Math.toDegrees(currPose.h));
        packet.addData("odometry", odometry);
        allHardware.forEach(h -> h.update(packet));

        packet.addSpace();
        if(pathCache.isEmpty()) {
            packet.addData("path empty","");
        } else {
            packet.addData("point amts", pathCache.size());
            packet.addData("current path name", pathCache.name);
            packet.addData("current target", pathCache.get(0).toString());
            double[] x = new double[pathCache.initialPoints.size()];
            double[] y = new double[pathCache.initialPoints.size()];
            int index = 0;
            for(PathPoints.BasePathPoint e : pathCache.initialPoints) {
                x[index] = e.y;
                y[index] = -e.x;
                packet.fieldOverlay().setFill("green").fillCircle(x[index],y[index],2);
                index++;
            }
            packet.fieldOverlay()
                    .setStroke("red")
                    .setStrokeWidth(1)
                    .strokePolyline(x,y);
        }
        Point dashPoint = currPose.dbNormalize(); // ok -(-y) looks stupid but i do it to get rid of the warning istg im not that stupid
        packet.fieldOverlay()
                .setFill("blue")
                .fillCircle(dashPoint.x, dashPoint.y, 3)
                .setStroke("purple")
                .setStrokeWidth(1)
                .strokeLine(dashPoint.x, dashPoint.y, dashPoint.x + 10*Math.sin(currPose.h), dashPoint.y - 10*Math.cos(currPose.h));
        if(isInit) updateDashboard();
    }

    private void updateDashboard() {
        if(dashboard != null)
            dashboard.sendTelemetryPacket(packet);
    }

    private void debugControl() {
        if(gamepad1.left_trigger > 0.5) {
            pathStopped = true;
        } else if(gamepad1.right_trigger > 0.5) {
            pathStopped = false;
        }
        if (System.currentTimeMillis() - lastManualUpdate > 50) {
            currPose = new Pose(
                    currPose.x + sgn(gamepad1.left_stick_x),
                    currPose.y - sgn(gamepad1.left_stick_y),
                    angleWrap(currPose.h + (sgn(-gamepad1.right_stick_x) * (Math.PI / 10)))
            );
            lastManualUpdate = System.currentTimeMillis();
        }
        if(debugging && !pathStopped && !pathCache.isEmpty()) {
            double elapsed = (System.currentTimeMillis() - lastAutoUpdate)/1000.0;
            lastAutoUpdate = System.currentTimeMillis();
            if(elapsed > 1) return;

            double radius = debugSpeeds.hypot();
            double theta = currPose.h + debugSpeeds.atan() - Math.PI / 2;
            currPose = currPose.add(new Pose(
                    radius * Math.cos(theta) * elapsed * 500 * 0.2,
                    radius * Math.sin(theta) * elapsed * 500 * 0.2,
                    driveTrain.powers.h * elapsed * 10  / (2 * Math.PI)));

            debugSpeeds = ((debugSpeeds.add(new Pose(
                    Range.clip((driveTrain.powers.x - debugSpeeds.x)/0.2,-1,1) * elapsed,
                    Range.clip((driveTrain.powers.y - debugSpeeds.y)/0.2,-1,1) * elapsed,
                    Range.clip((driveTrain.powers.h - debugSpeeds.h)/0.2,-1,1) * elapsed
            ))).scale(1.0 - elapsed));
        }
    }
}















