package org.firstinspires.ftc.teamcode.treamcodde.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Util {
    public static Pose2d changeSide(Pose2d pose) {
        return new Pose2d(pose.getX(), pose.getY(), pose.getHeading() * -1 );
    }
}
