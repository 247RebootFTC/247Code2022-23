//Color Sensor

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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
public class BasicColorSensor extends LinearOpMode
{
    OpenCvWebcam webcam;
    ColorDetectionPipeline pipeline = new ColorDetectionPipeline();

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("error");
                telemetry.update();
            }
        });



        waitForStart();

        String color = pipeline.getAnalysis();

        while (opModeIsActive()) {
            telemetry.addData("Analysis", color);

            telemetry.addData("Cb Value:", pipeline.getCb());
            telemetry.addData("Cr Value:", pipeline.getCr());
            telemetry.update();

            //Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }


    public static class ColorDetectionPipeline extends OpenCvPipeline
    {
        //Sets color of the rectangle to indicate if it sees a certain color
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar RED = new Scalar(255, 0, 0);

        //Sets viewing box position
        static final Point REGION_TOPLEFT_ANCHOR_POINT = new Point(140,100);
        static final int REGION_WIDTH = 40;
        static final int REGION_HEIGHT = 40;

        /*
           Points which actually define the sample region rectangles, derived from above values

           Example of how points A and B work to define a rectangle

             ------------------------------------
             | (0,0) Point A                    |
             |                                  |
             |                                  |
             |                                  |
             |                                  |
             |                                  |
             |                                  |
             |                  Point B (70,50) |
             ------------------------------------

         */

        //Creates the rectangle
        Point region_pointA = new Point(
                REGION_TOPLEFT_ANCHOR_POINT.x,
                REGION_TOPLEFT_ANCHOR_POINT.y);
        Point region_pointB = new Point(
                REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);


        Mat region_Cb, region_Cr;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        Mat Cr = new Mat();

        //Creates variables for the Cb and Cr values
        private int Cb_val, Cr_val;

        private String position;

        void inputToCb(Mat input)
        {
            //Converts color from RGB to YCbCr
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            //Extracts the Cb value
            Core.extractChannel(YCrCb, Cb, 2);
        }

        void inputToCr(Mat input)
        {
            //Converts color from RGB to YCbCr
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            //Extracts the Cr value
            Core.extractChannel(YCrCb, Cr, 1);
        }

        @Override
        //Reads the value in the viewing screen
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);
            inputToCr(firstFrame);

            region_Cb = Cb.submat(new Rect(region_pointA, region_pointB));
            region_Cr = Cr.submat(new Rect(region_pointA, region_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);
            //Gets average Cb value in the region
            Cb_val = (int) Core.mean(region_Cb).val[0];

            inputToCr(input);
            //Gets average Cr value in the region
            Cr_val = (int) Core.mean(region_Cr).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region_pointA, // First point which defines the rectangle
                    region_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    5); // Thickness of the rectangle lines

            if((Cb_val > 125)&&(Cr_val > 110)) // Was it from region 1?
            {
                position = "PURPLE"; // Record our analysis
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region_pointA, // First point which defines the rectangle
                        region_pointB, // Second point which defines the rectangle
                        BLUE, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if((Cb_val < 120)&&(Cr_val > 125))
            {
                position = "ORANGE"; // Record our analysis
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region_pointA, // First point which defines the rectangle
                        region_pointB, // Second point which defines the rectangle
                        RED, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if((Cb_val < 130)&&(Cr_val < 130))
            {
                position = "GREEN"; // Record our analysis

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region_pointA, // First point which defines the rectangle
                        region_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            return input;
        }
        //Returns position
        public String getAnalysis()
        {
            return position;
        }
        //Returns Cr value
        public int getCr()
        {
            return Cr_val;
        }
        //Returns Cb value
        public int getCb()
        {
            return Cb_val;
        }
    }
}