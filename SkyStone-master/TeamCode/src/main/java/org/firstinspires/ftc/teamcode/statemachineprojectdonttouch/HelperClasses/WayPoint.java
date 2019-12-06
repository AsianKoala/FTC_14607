package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses;

public class WayPoint {
    public double targetX;
    public double targetY;
    public double movementSpeed;
    public double pointAngle;
    public double pointSpeed;
    public double slowDownRadians;
    public double slowDownErrorMax;


    public WayPoint() {
         this.targetX = 0;
         this.targetY = 0;
         this.movementSpeed = 0;
         this.pointAngle = 0;
         this.pointSpeed = 0;
         this.slowDownRadians = 0;
         this.slowDownErrorMax = 0;
    }



    public WayPoint(double targetX, double targetY, double movementSpeed, double pointAngle, double pointSpeed, double slowDownRadians, double slowDownErrorMax) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.movementSpeed = movementSpeed;
        this.pointAngle = pointAngle;
        this.pointSpeed = pointSpeed;
        this.slowDownRadians = slowDownRadians;
        this.slowDownErrorMax = slowDownErrorMax;
    }


    public movementTarget getMovementTarget() {
        return new movementTarget(targetX, targetY, pointAngle);
    }
}
