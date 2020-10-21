package org.firstinspires.ftc.teamcode.HelperClasses;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import static java.lang.Math.*;

public  class GLOBALS {

    public  final static double flipperHome =  0.12; // todo : WAS 0.15
    public final  static double flipperOut = 0.8513; // todo : WAS 0.8513
    public  final static double flipperBetween = 0.27; // tODO: WAS 0.3 todo: after first chance it was 0.28
    public final static double capUp = .9;
    public final static double capBetween = 0.6;
    public final static double capDown = .1;
    public   static double rotaterHome = 0.279;
    public  static double rotaterOut = 0.95;
    public final static double gripperHome = 0.22; //
    public final static double gripperGrip = 0.82; // TODO: WAS 0.19


    public  static double P = 30; // todo: 12 15
    public  static double I = 0;
    public  static double D = 0;




    public static double   psuedoHomer = -50;





    public static SKYSTONE_POSITION ourSkystonePosition = SKYSTONE_POSITION.MIDDLE;
    public static double maxLiftHeightEncoderTicks = -1200; //TODO


    public enum SKYSTONE_POSITION {
        LEFT,
        MIDDLE,
        RIGHT
    }







    public static double AngleWrap(double angle) {
        while(angle <= -Math.PI) {
            angle += 2 * Math.PI;
        }

        while(angle > Math.PI) {
            angle -= 2 * Math.PI;
        }

        return angle;
    }



    private static final double odometryWheelRadius = 1.1811;
    private static final double TICKS_PER_REV = 2048; // TODO: recheck this
    /**
     * for odometry bruv
     * @param ticks
     * @return
     */
    public static double encoderTicksToInches(double ticks) {
        return odometryWheelRadius * 2 * Math.PI * ticks / TICKS_PER_REV;
    }






    public static double horizontalExtendHome = 0.1389;
    public static double horizontalExtendOut = 0.7187;
    public static double horizontalExtendFeed = 0.500;
    public static double rightHookGrip = 0.7414;
    public static double leftHookGrip = 0.1595;
    public static double leftHookHome = 0.4545;
    public static double rightHookHome = 0.4460;
    public static double capstoneDeployerOut = 0.6020;
    public static double capstoneDeployerHome = 0.9245;
    public static double frontGripperOpen = 0.3591;
    public static double frontGripperClosed = 0.047;
    public static double backGripperOpen = 0.230; // 0.5741
    public static double backGripperClosed = 0.6023; // 0.933
    public static double backGripperCap = 0.883; // 0.5
    public static double parkingDeployerOut = 0.5;
    public static double parkingDeployerIn = 0.5;

    public static double backClawFlipperHome = 0.7720; //
    public static double backClawFlipperAntiHome = 0.9922; //
    public static double backClawFlipperUp = 0.8742; //
    public static double backClawFlipperTravel = 0.7316; // 0.7215

    public static double backClawFlipperDown = 0.6800; //
    public static double backClawGripperHome = 0.1808; //
    public static double backClawGripperAntiHome = 0.1683;
    public static double backClawGripperOpen = 0.1083; // 0.1683
    public static double backClawGripperClosed = 0.32; // 351 2955
    public static double backClawRotaterHome = 0.07305; //
    public static double backClawRotaterOut = 0.3873; //
    public static double backClawRotaterBack = 0.9065; //

    public static double frontClawRotaterHome = 0.8751; //
    public static double frontClawRotaterOut = 0.5323; //
    public static double frontClawRotaterBack = 0.0584; //

    // swap flipper and gripper




    


}
