/* Copyright (c) 2020 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.robotcontroller.external.samples

import android.graphics.Bitmap
import android.graphics.ImageFormat
import android.os.Handler
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession.StatusCallback
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer
import org.firstinspires.ftc.robotcore.internal.system.Deadline
import java.io.File
import java.io.FileOutputStream
import java.io.IOException
import java.lang.RuntimeException
import java.util.Locale
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.TimeUnit

/**
 * This OpMode illustrates how to open a webcam and retrieve images from it. It requires a configuration
 * containing a webcam with the default name ("Webcam 1"). When the opmode runs, pressing the 'A' button
 * will cause a frame from the camera to be written to a file on the device, which can then be retrieved
 * by various means (e.g.: Device File Explorer in Android Studio; plugging the device into a PC and
 * using Media Transfer; ADB; etc)
 */
@TeleOp(name = "Concept: Webcam", group = "Concept")
@Disabled
class ConceptWebcam() : LinearOpMode() {
    /**
     * State regarding our interaction with the camera
     */
    private lateinit var cameraManager: CameraManager
    private lateinit var cameraName: WebcamName
    private var camera: Camera? = null
    private var cameraCaptureSession: CameraCaptureSession? = null

    /**
     * The queue into which all frames from the camera are placed as they become available.
     * Frames which are not processed by the OpMode are automatically discarded.
     */
    private lateinit var frameQueue: EvictingBlockingQueue<Bitmap>

    /**
     * State regarding where and how to save frames when the 'A' button is pressed.
     */
    private var captureCounter = 0
    private val captureDirectory = AppUtil.ROBOT_DATA_DIR

    /**
     * A utility object that indicates where the asynchronous callbacks from the camera
     * infrastructure are to run. In this OpMode, that's all hidden from you (but see [.startCamera]
     * if you're curious): no knowledge of multi-threading is needed here.
     */
    private lateinit var callbackHandler: Handler

    // ----------------------------------------------------------------------------------------------
    // Main OpMode entry
    // ----------------------------------------------------------------------------------------------
    override fun runOpMode() {
        callbackHandler = CallbackLooper.getDefault().handler
        cameraManager = ClassFactory.getInstance().cameraManager
        cameraName = hardwareMap.get(WebcamName::class.java, "Webcam 1")
        initializeFrameQueue(2)
        AppUtil.getInstance().ensureDirectoryExists(captureDirectory)
        try {
            openCamera()
            if (camera == null) return
            startCamera()
            if (cameraCaptureSession == null) return
            telemetry.addData(">", "Press Play to start")
            telemetry.update()
            waitForStart()
            telemetry.clear()
            telemetry.addData(">", "Started...Press 'A' to capture frame")
            var buttonPressSeen = false
            var captureWhenAvailable = false
            while (opModeIsActive()) {
                val buttonIsPressed = gamepad1.a
                if (buttonIsPressed && !buttonPressSeen) {
                    captureWhenAvailable = true
                }
                buttonPressSeen = buttonIsPressed
                if (captureWhenAvailable) {
                    val bmp = frameQueue!!.poll()
                    if (bmp != null) {
                        captureWhenAvailable = false
                        onNewFrame(bmp)
                    }
                }
                telemetry.update()
            }
        } finally {
            closeCamera()
        }
    }

    /**
     * Do something with the frame
     */
    private fun onNewFrame(frame: Bitmap) {
        saveBitmap(frame)
        frame.recycle() // not strictly necessary, but helpful
    }

    // ----------------------------------------------------------------------------------------------
    // Camera operations
    // ----------------------------------------------------------------------------------------------
    private fun initializeFrameQueue(capacity: Int) {
        /** The frame queue will automatically throw away bitmap frames if they are not processed
         * quickly by the OpMode. This avoids a buildup of frames in memory  */
        frameQueue = EvictingBlockingQueue(ArrayBlockingQueue(capacity))
        frameQueue!!.setEvictAction { frame ->
            // RobotLog.ii(TAG, "frame recycled w/o processing");
            frame.recycle() // not strictly necessary, but helpful
        }
    }

    private fun openCamera() {
        if (camera != null) return // be idempotent
        val deadline = Deadline(secondsPermissionTimeout.toLong(), TimeUnit.SECONDS)
        camera = cameraManager!!.requestPermissionAndOpenCamera(deadline, cameraName, null)
        if (camera == null) {
            error("camera not found or permission to use not granted: %s", (cameraName)!!)
        }
    }

    private fun startCamera() {
        if (cameraCaptureSession != null) return // be idempotent
        /** YUY2 is supported by all Webcams, per the USB Webcam standard: See "USB Device Class Definition
         * for Video Devices: Uncompressed Payload, Table 2-1". Further, often this is the *only*
         * image format supported by a camera  */
        val imageFormat = ImageFormat.YUY2
        /** Verify that the image is supported, and fetch size and desired frame rate if so  */
        val cameraCharacteristics = cameraName!!.cameraCharacteristics
        if (!contains(cameraCharacteristics.androidFormats, imageFormat)) {
            error("image format not supported")
            return
        }
        val size = cameraCharacteristics.getDefaultSize(imageFormat)
        val fps = cameraCharacteristics.getMaxFramesPerSecond(imageFormat, size)

        /** Some of the logic below runs asynchronously on other threads. Use of the synchronizer
         * here allows us to wait in this method until all that asynchrony completes before returning.  */
        val synchronizer = ContinuationSynchronizer<CameraCaptureSession?>()
        try {
            /** Create a session in which requests to capture frames can be made  */
            camera!!.createCaptureSession(
                Continuation.create(
                    callbackHandler,
                    object : CameraCaptureSession.StateCallbackDefault() {
                        override fun onConfigured(session: CameraCaptureSession) {
                            try {
                                /** The session is ready to go. Start requesting frames  */
                                val captureRequest =
                                    camera!!.createCaptureRequest(imageFormat, size, fps)
                                session.startCapture(
                                    captureRequest,
                                    CameraCaptureSession.CaptureCallback { session, request, cameraFrame ->
                                        /** A new frame is available. The frame data has *not* been copied for us, and we can only access it
                                         * for the duration of the callback. So we copy here manually.  */
                                        /** A new frame is available. The frame data has *not* been copied for us, and we can only access it
                                         * for the duration of the callback. So we copy here manually.  */
                                        val bmp = captureRequest.createEmptyBitmap()
                                        cameraFrame.copyToBitmap(bmp)
                                        frameQueue.offer(bmp)
                                    },
                                    Continuation.create(
                                        callbackHandler,
                                        object : StatusCallback {
                                            override fun onCaptureSequenceCompleted(
                                                session: CameraCaptureSession,
                                                cameraCaptureSequenceId: CameraCaptureSequenceId,
                                                lastFrameNumber: Long
                                            ) {
                                                RobotLog.ii(
                                                    TAG,
                                                    "capture sequence %s reports completed: lastFrame=%d",
                                                    cameraCaptureSequenceId,
                                                    lastFrameNumber
                                                )
                                            }
                                        }
                                    )
                                )
                                synchronizer.finish(session)
                            } catch (e: CameraException) {
                                RobotLog.ee(TAG, e, "exception starting capture")
                                error("exception starting capture")
                                session.close()
                                synchronizer.finish(null)
                            } catch (e: RuntimeException) {
                                RobotLog.ee(TAG, e, "exception starting capture")
                                error("exception starting capture")
                                session.close()
                                synchronizer.finish(null)
                            }
                        }
                    }
                )
            )
        } catch (e: CameraException) {
            RobotLog.ee(TAG, e, "exception starting camera")
            error("exception starting camera")
            synchronizer.finish(null)
        } catch (e: RuntimeException) {
            RobotLog.ee(TAG, e, "exception starting camera")
            error("exception starting camera")
            synchronizer.finish(null)
        }
        /** Wait for all the asynchrony to complete  */
        try {
            synchronizer.await()
        } catch (e: InterruptedException) {
            Thread.currentThread().interrupt()
        }
        /** Retrieve the created session. This will be null on error.  */
        cameraCaptureSession = synchronizer.value!!
    }

    private fun stopCamera() {
        if (cameraCaptureSession != null) {
            cameraCaptureSession!!.stopCapture()
            cameraCaptureSession!!.close()
            cameraCaptureSession = null
        }
    }

    private fun closeCamera() {
        stopCamera()
        if (camera != null) {
            camera!!.close()
            camera = null
        }
    }

    // ----------------------------------------------------------------------------------------------
    // Utilities
    // ----------------------------------------------------------------------------------------------
    private fun error(msg: String) {
        telemetry.log().add(msg)
        telemetry.update()
    }

    private fun error(format: String, vararg args: Any) {
        telemetry.log().add(format, *args)
        telemetry.update()
    }

    private fun contains(array: IntArray, value: Int): Boolean {
        for (i: Int in array) {
            if (i == value) return true
        }
        return false
    }

    private fun saveBitmap(bitmap: Bitmap) {
        val file = File(
            captureDirectory,
            String.format(Locale.getDefault(), "webcam-frame-%d.jpg", captureCounter++)
        )
        try {
            FileOutputStream(file).use { outputStream ->
                bitmap.compress(Bitmap.CompressFormat.JPEG, 100, outputStream)
                telemetry.log().add("captured %s", file.getName())
            }
        } catch (e: IOException) {
            RobotLog.ee(TAG, e, "exception in saveBitmap()")
            error("exception saving %s", file.name)
        }
    }

    companion object {
        // ----------------------------------------------------------------------------------------------
        // State
        // ----------------------------------------------------------------------------------------------
        private val TAG = "Webcam Sample"

        /**
         * How long we are to wait to be granted permission to use the camera before giving up. Here,
         * we wait indefinitely
         */
        private val secondsPermissionTimeout = Int.MAX_VALUE
    }
}
