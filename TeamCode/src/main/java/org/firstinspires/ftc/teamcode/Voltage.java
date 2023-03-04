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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * This sample demonstrates a basic (but battle-tested and essentially
 * 100% accurate) method of detecting the skystone when lined up with
 * the sample regions over the first 3 stones.
 */

@Disabled
@Autonomous
public class Voltage extends LinearOpMode {

    //Declare Drive Motors
    private DcMotor motorfrontLeft;
    private DcMotor motorfrontRight;
    private DcMotor motorbackLeft;
    private DcMotor motorbackRight;

    //The Time Object
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        telemetry.addData(">", "DRAGON: FIIIIIIIIIIIIIIIIRE");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            //Initialize Drive Motors
            motorfrontLeft = hardwareMap.dcMotor.get("motorfrontLeft");
            motorfrontRight = hardwareMap.dcMotor.get("motorfrontRight");
            motorbackLeft = hardwareMap.dcMotor.get("motorbackLeft");
            motorbackRight = hardwareMap.dcMotor.get("motorbackRight");

            //Initialize Drive Motors' Directions
            motorfrontLeft.setDirection(DcMotor.Direction.REVERSE);
            motorfrontRight.setDirection(DcMotor.Direction.FORWARD);
            motorbackLeft.setDirection(DcMotor.Direction.REVERSE);
            motorbackRight.setDirection(DcMotor.Direction.FORWARD);

            VoltageSensor voltSensor = hardwareMap.voltageSensor.get("Motor Controller 1");


            telemetry.addLine(String.valueOf(voltSensor.getVoltage()));
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    /** FUNCTIONS */

    /** TIME-BASED */

    public void moveForward(double time){
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            motorfrontLeft.setPower(1);
            motorfrontRight.setPower(1);
            motorbackLeft.setPower(1);
            motorbackRight.setPower(1);
        }
    }

    public void moveBackward(double time){
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            motorfrontLeft.setPower(-1);
            motorfrontRight.setPower(-1);
            motorbackLeft.setPower(-1);
            motorbackRight.setPower(-1);
        }
    }

    public void turnLeft(double time){
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            motorfrontLeft.setPower(-1);
            motorfrontRight.setPower(1);
            motorbackLeft.setPower(-1);
            motorbackRight.setPower(1);
        }
    }

    public void turnRight(double time){
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            motorfrontLeft.setPower(1);
            motorfrontRight.setPower(-1);
            motorbackLeft.setPower(1);
            motorbackRight.setPower(-1);
        }
    }

    public void strafeLeft(double time){
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            motorfrontLeft.setPower(-0.5);
            motorfrontRight.setPower(0.5);
            motorbackLeft.setPower(0.5);
            motorbackRight.setPower(-0.5);
        }
    }

    public void strafeRight(double time){
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            motorfrontLeft.setPower(0.5);
            motorfrontRight.setPower(-0.5);
            motorbackLeft.setPower(-0.5);
            motorbackRight.setPower(0.5);
        }
    }

    public void stop(double time){
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            motorfrontLeft.setPower(0);
            motorfrontRight.setPower(0);
            motorbackLeft.setPower(0);
            motorbackRight.setPower(0);
        }
    }
}