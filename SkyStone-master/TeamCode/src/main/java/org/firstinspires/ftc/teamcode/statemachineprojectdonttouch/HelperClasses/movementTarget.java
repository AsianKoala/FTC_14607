package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses;

public class movementTarget {

        double x;
        double y;
        double heading;

        public movementTarget(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        public boolean equals(movementTarget target2) {
            return (x == target2.x) && (y == target2.y) && heading == target2.heading;
        }

}
