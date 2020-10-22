package org.firstinspires.ftc.teamcode.test.auto;

import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;



public class RingDetector extends TunableLinearOpMode {

    @Override
    public void runOpMode() {
        // lol
    }


    /*
    *
    * Most of the stuff written here was got from the
    * documentation in the opencv library (linked below)
    * easyopencv is a nice lib that allows for use of opencv with ftc without too much bullshit
    *
    * @link https://docs.opencv.org/3.1.0/
    * */
    private static class RingDetectorPipeline extends OpenCvPipeline {

        private enum RingAmount {
            DEBUG, // checks if the detector is being stupid
            NONE,
            ONE,
            FOUR,
        }

        // color constants
        static final Scalar BLUE = new Scalar(0,0,255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        // constants for defining the area of rectangles
        // TODO: actually find values for these constants
        static final Point FOUR_RING_TOP_LEFT_ANCHOR = new Point(100, 100);
        static final Point ONE_RING_TOP_LEFT_ANCHOR = new Point(100, 50);
        static final int REC_WIDTH = 20;
        static final int REC_HEIGHT = 10;

        // using the constants we calculate the points actually used for the rectangles
        // point a would be the top left point, point b would be the bottom right (creating a diagonal)

        Point four_ring_pointA = FOUR_RING_TOP_LEFT_ANCHOR;
        Point four_ring_pointB = new Point(FOUR_RING_TOP_LEFT_ANCHOR.x + REC_WIDTH, FOUR_RING_TOP_LEFT_ANCHOR.y + REC_HEIGHT);
        Point one_ring_pointA = ONE_RING_TOP_LEFT_ANCHOR;
        Point one_ring_pointB = new Point(ONE_RING_TOP_LEFT_ANCHOR.x + REC_WIDTH, ONE_RING_TOP_LEFT_ANCHOR.y + REC_HEIGHT);

        // computation vars
        // region1 is bound by the first the points
        // region2 is bound by the second 2
        Mat region1_Cb, region2_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;

        private RingAmount ringAmt = RingAmount.DEBUG;

        // turns rgb input into YCrCb
        void rgbToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }


        @Override
        public void init(Mat firstFrame) {

            rgbToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(four_ring_pointA, four_ring_pointB));
            region2_Cb = Cb.submat(new Rect(one_ring_pointA, one_ring_pointB));

        }


    }

}
