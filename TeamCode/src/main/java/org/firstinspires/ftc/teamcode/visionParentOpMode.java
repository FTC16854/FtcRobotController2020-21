/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name="VisionAuto1", group="Linear Opmode")
public class visionParentOpMode extends ParentOpMode
{
    OpenCvCamera phoneCam;
    RingDeterminationPipeline pipeline;

    @Override
    public void runOpMode()
    {
        initialize();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"), cameraMonitorViewId);
        pipeline = new RingDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        waitForStart();


        runtime.reset();
        while (opModeIsActive())
        {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);

            //Autnomous Code start
            //Branch into different autonomous modes using vision data
            switch (pipeline.position){
                //If one ring detected
                case ONE:
                    //Update telemetry
                    telemetry.addData("Rings_Detected: ", "1");
                    telemetry.update();
                    //sleep(30000);
                    phoneCam.closeCameraDevice();
                    //Run specific autonomous
                    Driveandshoot();
                    oneRings();
                    stopDrive();
                    saveHeading();
                    break;

                 //If four rings detected
                case FOUR:
                    telemetry.addData("Rings_Detected: ", "4");
                    telemetry.update();
                    //sleep(30000);
                    phoneCam.closeCameraDevice();
                    Driveandshoot();
                    fourRings();
                    stopDrive();
                    saveHeading();
                    break;

                //If no rings detected
                case NONE:
                    telemetry.addData("Rings_Detected: ", "0");
                    telemetry.update();
                    //sleep(30000);
                    phoneCam.closeCameraDevice();
                    Driveandshoot();
                    zeroRings();
                    stopDrive();
                    saveHeading();
                    break;


            }
            break;
        }
    }
    public void fourRings(){
        driveInchesVertical(-45,1);
        rotateToHeading(.3,-75,'r');
        rotateToHeading(.1,-90,'r');
        driveInchesVertical(-1,1);
        autoLiftDown();
        sleep(1000);
        autoClawOpen();
        driveInchesVertical(12,1);
        driveInchesHorizontal(-35,1);
        rotateToHeading(.3,-15,'l');
        rotateToHeading(.1,0,'l');
    }

    public void oneRings(){
        driveInchesVertical(-17,1);
        driveInchesHorizontal(-1,1);
        autoLiftDown();
        sleep(1000);
        autoClawOpen();
        driveInchesVertical(6,1);
        rotateToHeading(.1,0,'l');
        rotateToHeading(.1,0,'r');
    }

    public void zeroRings(){
        rotateToHeading(.3,-75,'r');
        rotateToHeading(.1,-90,'r');
        driveInchesHorizontal(4,1);
        driveInchesVertical(-10,1);
        autoLiftDown();
        sleep(1000);
        autoClawOpen();
        driveInchesVertical(12,1);
        rotateToHeading(.3,-15,'l');
        rotateToHeading(.1,0,'l');
    }

    public void Driveandshoot(){
        autoClawClose();
        autoLiftUp();

        sleep(1000);
        driveInchesVertical(-55, 1);
        driveInchesHorizontal(24, 1);
        rotateToHeading(.1,0,'l');
        rotateToHeading(.1,0,'r');
        for (int i = 0; i < 3; i++) {
            shootAuto(0.507);
        }
        shooterStop();

    }

    public static class RingDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the number of rings in the stack
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
    //    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181,98);
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(80,185);

        static final int REGION_WIDTH = 40;
        static final int REGION_HEIGHT = 40;
//        static final int REGION_WIDTH = 60;
//        static final int REGION_HEIGHT = 40;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }



}