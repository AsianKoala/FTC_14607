package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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


@TeleOp
public class CBRingDetector extends TunableLinearOpMode {

    OpenCvInternalCamera phoneCam;
    RingDetectorPipeline pipeline;

    @Override
    public void runOpMode() {

        // literally just last yrs code for starting up the camera and initializing everything
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new RingDetectorPipeline();
        phoneCam.setPipeline(pipeline);

        // fixes the problems we had last year with the camera preview on the phone not being in landscape or wtever
        // also starts the camera
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        // init loop
        while(!isStarted() && !isStopRequested()) {
            telemetry.addData("Ready.", 0);
            telemetry.update();
        }

        while (opModeIsActive())
        {
            telemetry.addData("Rings: ", pipeline.getRingAmount());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }


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

        // color constants used for rectangles drawn on the phone like last yr
        static final Scalar BLUE = new Scalar(0,0,255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar RED = new Scalar(255, 0, 0);

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
        int avg1, avg2;

        private volatile RingAmount ringAmt = RingAmount.DEBUG;

        // turns rgb input into YCrCb
        void rgbToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        // just makes sure shit is initialized so it doesn't break later on
        @Override
        public void init(Mat firstFrame) {
            rgbToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(four_ring_pointA, four_ring_pointB));
            region2_Cb = Cb.submat(new Rect(one_ring_pointA, one_ring_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            /*
            * - Explanation of whats happening here - (I'm basically a god commenter at this point amiright)
            * We convert the colors from RGB to YCrCb
            * This is because the vision code can better view the
            * region without being affected by darkness/brightness since that
            * will be in the Y channel instead of the Cb or Cr channel.
            * In rgb this would've been reflected over all 3 channels, R, G, and B
            * This link was really good at explaining how YCrCb compares to other color spaces
            * It also shows the differences in OpenCV code
            * https://www.learnopencv.com/color-spaces-in-opencv-cpp-python/
            *
            * We only need the Cb channel from the color-space since that reflects
            * the yellow-orange color the best
            * */

            // changes color space
            rgbToCb(input);

            // get avg of each region
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];

            /*
            * draws rectangles on the phone like last year's vision code
            * honestly the most retarded part of the code
            * it took me so long to manually estimate where the rectangles should be
            * until i realized that i could simply make rectangles based on the initial points that i made
            * yeah.. kinda stupid of me
            *
            * since the rectangles will be overlapped i thought its better to make them a separate color/thickness
            */

            // big region for 4 rings
            Imgproc.rectangle(
                    input, // what to draw on
                    four_ring_pointA, // top left pt
                    four_ring_pointB, // bottom right pt
                    BLUE, // color
                    2
            );

            Imgproc.rectangle(
                    input,
                    one_ring_pointA,
                    one_ring_pointB,
                    GREEN,
                    1 // i guess just make it 1 since its inside of the other region? idk
            );

            // TODO: add rectangle showing correct region on phone
            // TODO: find a threshold for no rings
            final int threshold = 0;
            if(avg1 > avg2) { // most of the region is yellow-orange
                ringAmt = RingAmount.FOUR;
            }
            else if(avg2 > threshold) { // some (most likely low) part of the region is yellow-orange
                ringAmt = RingAmount.ONE;
            }
            else { // near 0 of the region is yellow-orange (needs to be adjusted with threshold)
                ringAmt = RingAmount.NONE;
            }

            return input;
        }

        public RingAmount getRingAmount() {
            return ringAmt;
        }
    }

}
