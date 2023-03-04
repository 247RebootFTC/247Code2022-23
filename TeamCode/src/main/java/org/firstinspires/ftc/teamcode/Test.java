/* Copyright (c) 2019 FIRST. All rights reserved.
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

//package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import java.util.Iterator;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.security.Guard;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
//import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Freight Frenzy game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Test", group = "Blue Autonomi")
@Disabled
public class Test extends LinearOpMode {

    /* Encoder Specific defines */
    private static final double COUNTS_PER_MOTOR_REV    = 675; //537.6   // eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION    = 1 ;     // 1 IF MOUNTED DIRECTLY TO MOTOR IF NOT-  
    //-divide the number of teeth on the wheel sprocket-
    //-by the number of teeth on the motor sprocket
    //This is < 1.0 if geared UP //this is for wheels only, make new variables for other motors
    private static final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference //this is for wheels only, make new variables for other motors
    private static final double INTAKE_DIAMETER_INCHES  = 3.0 ;     // For figuring circumference of intake
    private static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415); //this is for wheels only, make new variables for other motors
    //private static final double COUNTS_PER_INCH_CAL     = COUNTS_PER_INCH + 300;
    private static final double INTAKE_COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (INTAKE_DIAMETER_INCHES * 3.1415);
    private static final double DRIVE_SPEED             = 0.4;//0.4, 1.0; //this is for wheels only, make new variables for other motors
    private static final double MAX_SPEED               = 0.6;
    private static final double TURN_SPEED              = 0.4; //this is for wheels only, make new variables for other motors



    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    /*private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            //"Marker"
    };*/

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AdPYch7/////AAABmaJmIY+IeEWsjN7JO1LMCWaD1YmM0SXe9dyOwdPyLskTRmr0i1qmD3GE4cNah9UDNv5D99W4Uu2VERcdwq2bgIRQSEuoa6mpmX86ropCn1rLJ3dpgTvGB9r1tcYsxiSqK6GRTRQPPcyxDg7ZIbxU8IZqyNoszZSgwQ50gjYN2/uABP0n7HOkTmU/nWYLgI2A+Vx5ywIm4S6xLr4/AKYfQS2jG+QBv7qqW2549r2/mW2xpELBAAL7CVNx35DGxuWOdelHYtsJEb06Tp3VAMg2M22luWT4yTYPZSfSu9FcQX5qi7nuJSN9+awUzLm16g+QG1Nq3y4chztDuwNuqPko9uSwoiz04BZuh6TUdD8Ys10p";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /*
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    //private TFObjectDetector tfod;

    //Declare Drive Motors
    private DcMotor motorfrontLeft;
    private DcMotor motorfrontRight;
    private DcMotor motorbackLeft;
    private DcMotor motorbackRight;

    //Declare Mechanism Motors
    /*private DcMotor IntakeMotor;
    private DcMotor CarouselMotor;
    private DcMotor ArmMotor;*/


    //Declare servos
   /* private CRServo leftEye;
    private CRServo rightEye;*/

    //The Time Object
    private ElapsedTime runtime=new ElapsedTime();

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        //initTfod();

        /*
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        //if (tfod != null) {

            //tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            //tfod.setZoom(1.55555555555555, 16.0/9.0);
            //tfod.setZoom(2.5, 16.0/9.0);
        //}

        /* Wait for the game to begin */
        telemetry.addData(">", "Press Play");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {

            //Initialize drive motors
            motorfrontLeft = hardwareMap.dcMotor.get("motorfrontLeft");
            motorfrontRight = hardwareMap.dcMotor.get("motorfrontRight");
            motorbackLeft = hardwareMap.dcMotor.get("motorbackLeft");
            motorbackRight = hardwareMap.dcMotor.get("motorbackRight");

            //Initialize other motors
           /* IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
            CarouselMotor = hardwareMap.dcMotor.get("CarouselMotor");
            ArmMotor = hardwareMap.dcMotor.get("ArmMotor");

            //Initialize servos
            leftEye = hardwareMap.crservo.get("leftEye");
            rightEye = hardwareMap.crservo.get("rightEye");*/

            //runs the code twice so camera double checks for duck
            int runOnce = 2;
            int runTwice = 2;

            //will store how many objects are sensed by the camera
            int size;
            int size2;

            //Initialize drive motors' direction
            motorfrontLeft.setDirection(DcMotor.Direction.REVERSE);
            motorfrontRight.setDirection(DcMotor.Direction.FORWARD);
            motorbackLeft.setDirection(DcMotor.Direction.REVERSE);
            motorbackRight.setDirection(DcMotor.Direction.FORWARD);

            /*ArmMotor.setDirection(DcMotor.Direction.REVERSE);
            CarouselMotor.setDirection(DcMotor.Direction.REVERSE);
            IntakeMotor.setDirection(DcMotor.Direction.REVERSE);
*/
            /*while (opModeIsActive()&&(runOnce>0)) {
                if (tfod != null) {
                    if(runOnce==2) {
                        encoderDrive(DRIVE_SPEED, 11.5, 11.5);*/
                        /*for(int index = 0; index < 2; index++) {
                            moveEyes(0.4);
                            moveEyesOtherWay(0.4);
                        }*//*
                    }
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    size = updatedRecognitions.size();
                    if (updatedRecognitions != null) {
                        // step through the list of recognitions and display boundary info.
                        //shows how many objects are detected
                        telemetry.addData("# Object Detected", size);
                        telemetry.update();
                        Stop(1.5);
                        //if no objects detected and moving code hasn't happened, if statement executes
                        if((size==0) && (runOnce==1))
                        {
                            //moves to second barcode
                            //encoderStrafe(DRIVE_SPEED, -12.8, 12.8);
                            moveArmUpward(0.05);
                            Stop(0.01);
                            encoderDrive(TURN_SPEED, -6.5, 6.5);
                            encoderDrive(DRIVE_SPEED, -1.5, -1.5);
                            while(runTwice>0) {
                                runOnce--;
                                updatedRecognitions = tfod.getUpdatedRecognitions();
                                size2 = updatedRecognitions.size();
                                Stop(1.0);
                                //tells what has been detected
                                for (Recognition recognition2 : updatedRecognitions) {
                                    //executes if anything is detected and moving code has never run
                                    //2nd Barcode
                                    if((runTwice==1)&&(size2>0)) {
                                        //checks if item detected is a duck
                                        if((recognition2.getLabel()).equals("Duck")) {
                                            encoderDrive(TURN_SPEED, 6.5, -6.5);
                                            moveArmUpward(0.53);
                                            Stop(0.01);
                                            encoderDrive(TURN_SPEED, 8, -8);
                                            encoderDrive(DRIVE_SPEED, 19, 19);
                                            Outtake(3.0);
                                            Stop(0.01);
                                            encoderDrive(DRIVE_SPEED, -19, -19);
                                            encoderDrive(TURN_SPEED, -10.5, 10.5);
                                            moveArmDown(0.25);
                                            Stop(0.01);
                                            IntakeDuck(0.5);
                                            encoderDrive(DRIVE_SPEED, 10, 10);
                                            StopExceptWOIntake(0.5);
                                            IntakeDuck(0.5);
                                            Stop(0.01);
                                            encoderDrive(TURN_SPEED, 14, -14);
                                            moveArmUpward(0.56);
                                            Stop(0.01);
                                            encoderDrive(DRIVE_SPEED, 12, 12);
                                            OuttakeDuck(1.5);
                                            Stop(0.01);
                                            encoderDrive(DRIVE_SPEED, -5, -5);
                                            encoderDrive(TURN_SPEED, 8, -8);
                                            encoderStrafe(DRIVE_SPEED, 7, -7);
                                            encoderDrive(DRIVE_SPEED, -60, -60);*/
                                            /*IntakeDuck(1.0);
                                            encoderDrive(DRIVE_SPEED, 11, 11);
                                            StopExceptWOIntake(0.5);
                                            Stop(0.1);
                                            encoderDrive(TURN_SPEED, 19, -19);
                                            moveArmUpward(0.517);
                                            Stop(0.01);
                                            encoderDrive(DRIVE_SPEED, 9, 9);
                                            OuttakeDuck(3.0);
                                            Stop(0.01);
                                            encoderStrafe(DRIVE_SPEED, 20, -20);
                                            encoderDrive(DRIVE_SPEED, -60, -60);*/
                                            /*runTwice--;
                                        }
                                    }
                                }
                                //3rd Barcode/*
                                if((size2==0)&&(runTwice==1)){
                                    encoderDrive(TURN_SPEED, 6.5, -6.5);
                                    moveArmUpward(0.287);
                                    Stop(0.01);
                                    encoderDrive(TURN_SPEED, 8, -8);
                                    encoderDrive(DRIVE_SPEED, 18, 18);
                                    Outtake(3.0);
                                    Stop(0.01);*/
                                    /*encoderDrive(DRIVE_SPEED, -15, -15);
                                    encoderDrive(TURN_SPEED, 8, -8);
                                    encoderDrive(DRIVE_SPEED, -60, -60);*/
                                    /*encoderDrive(DRIVE_SPEED, -2, -2);
                                    encoderDrive(TURN_SPEED, -25, 25);
                                    moveArmDown(0.09);
                                    Stop(0.01);
                                    IntakeDuck(0.5);
                                    encoderDrive(DRIVE_SPEED, 11, 11);
                                    StopExceptWOIntake(0.5);
                                    IntakeDuck(0.5);
                                    Stop(0.01);
                                    moveArmUpward(0.287);
                                    Stop(0.01);
                                    encoderDrive(TURN_SPEED, 27.5, -27.5);
                                    encoderDrive(DRIVE_SPEED, 11, 11);
                                    OuttakeDuck(1.5);
                                    Stop(0.01);
                                    encoderDrive(DRIVE_SPEED, -10, -10);
                                    encoderDrive(TURN_SPEED, 11, -11);
                                    encoderStrafe(DRIVE_SPEED, 10, -10);
                                    encoderDrive(DRIVE_SPEED, -60, -60);*/
                                    /*encoderDrive(DRIVE_SPEED, -2.5, -2.5);
                                    encoderDrive(TURN_SPEED, 4.5, -4.5);
                                    moveArmUpward(0.287);
                                    Stop(0.01);
                                    encoderDrive(TURN_SPEED, 8, -8);
                                    encoderDrive(DRIVE_SPEED, 16, 16);
                                    encoderDrive(TURN_SPEED, 2.5, -2.5);
                                    Outtake(3.0);
                                    Stop(0.01);
                                    encoderDrive(DRIVE_SPEED, -5, -5);
                                    encoderDrive(TURN_SPEED, -30.5, 30.5);
                                    encoderStrafe(DRIVE_SPEED, 7, -7);
                                    moveArmDown(0.09);
                                    Stop(0.01);
                                    IntakeDuck(1.0);
                                    encoderDrive(DRIVE_SPEED, 12, 12);
                                    StopExceptWOIntake(0.5);
                                    Stop(0.1);
                                    encoderDrive(TURN_SPEED, 25, -25);
                                    moveArmUpward(0.287);
                                    Stop(0.01);
                                    encoderDrive(DRIVE_SPEED, 9, 9);
                                    OuttakeDuck(3.0);
                                    Stop(0.01);
                                    encoderStrafe(DRIVE_SPEED, 15, -15);
                                    encoderDrive(DRIVE_SPEED, -60, -60);*/
                                    /*runTwice--;
                                }
                                runTwice--;
                            }
                        }
                        //executes if code has never run and anything is detected
                        //1st Barcode
                        else if((runOnce==1)&&(size>0)) {
                            for (Recognition recognition : updatedRecognitions) {
                                if(recognition.getLabel().equals("Duck")) {
                                    moveArmUpward(1.0);
                                    Stop(0.01);
                                    encoderDrive(DRIVE_SPEED, 6, 6);
                                    encoderDrive(TURN_SPEED, 8, -8);
                                    encoderDrive(DRIVE_SPEED, 15, 15);
                                    encoderDrive(TURN_SPEED, 2.5, -2.5);
                                    Outtake(3.0);
                                    Stop(0.01);
                                    encoderDrive(TURN_SPEED, -3.5, 3.5);
                                    encoderDrive(DRIVE_SPEED, -19, -19);
                                    moveArmDown(0.54);
                                    Stop(0.01);
                                    encoderDrive(DRIVE_SPEED, 7, 7);
                                    IntakeDuck(0.5);
                                    encoderDrive(DRIVE_SPEED, 7, 7);
                                    StopExceptWOIntake(0.5);
                                    Stop(0.1);
                                    encoderDrive(TURN_SPEED, 3, -3);
                                    moveArmUpward(1.05);
                                    Stop(0.01);
                                    encoderDrive(DRIVE_SPEED, 6, 6);
                                    OuttakeDuck(3.0);
                                    Stop(0.01);
                                    encoderDrive(DRIVE_SPEED, -12, -12);
                                    encoderDrive(TURN_SPEED, 9.75, -9.75);
                                    encoderDrive(DRIVE_SPEED, -60, -60);
                                    runOnce--;
                                }
                            }
                        }
                        telemetry.update();
                    }
                    //makes runOnce one less so camera checks twice for any objects and moving code doesn't
                    //execute multiple times
                    runOnce--;
                }
            }*/
        }
        /*if (tfod != null) {
            tfod.shutdown();
        }*/
    }

    /** FUNCTIONS */

    /** TIME-BASED */
    /*public void CarouselSpin(double time){
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            CarouselMotor.setPower(1);
        }
    }
    public void Intake(double time){
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            IntakeMotor.setPower(1.0);
        }
    }
    public void IntakeDuck(double time){
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            IntakeMotor.setPower(0.5);
        }
    }
    public void Outtake(double time){
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            IntakeMotor.setPower(-0.5);
        }
    }
    public void OuttakeDuck(double time){
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            IntakeMotor.setPower(-1.0);
        }
    }
    public void moveArmUpward(double time){
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            ArmMotor.setPower(1);
        }
    }
    public void moveArmDown(double time){
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            ArmMotor.setPower(-1);
        }
    }
    public void moveEyes(double time){
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            leftEye.setPower(1);
            rightEye.setPower(-0.75);
        }
    }
    public void moveEyesOtherWay(double time){
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            leftEye.setPower(-1);
            rightEye.setPower(0.75);
        }
    }
    public void Stop(double time){
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            motorfrontLeft.setPower(0);
            motorfrontRight.setPower(0);
            motorbackLeft.setPower(0);
            motorbackRight.setPower(0);
            ArmMotor.setPower(0);
            IntakeMotor.setPower(0);
            CarouselMotor.setPower(0);
            leftEye.setPower(0);
            rightEye.setPower(0);
        }
    }
    public void StopExceptWOIntake(double time){
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            motorfrontLeft.setPower(0);
            motorfrontRight.setPower(0);
            motorbackLeft.setPower(0);
            motorbackRight.setPower(0);
            ArmMotor.setPower(0);
            CarouselMotor.setPower(0);
            leftEye.setPower(0);
            rightEye.setPower(0);
        }
    }*/
    /** ENCODERS */

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

    //Moves forwards & backwards and turns left & right
    /*public void encoderDrive(double speed,
                             double leftInches, double rightInches
    ) {
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfrontLeftTarget = motorfrontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newfrontRightTarget = motorfrontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newbackLeftTarget = motorbackLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newbackRightTarget = motorbackRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            motorfrontLeft.setTargetPosition(newfrontLeftTarget);
            motorfrontRight.setTargetPosition(newfrontRightTarget);
            motorbackLeft.setTargetPosition(newbackLeftTarget);
            motorbackRight.setTargetPosition(newbackRightTarget);

            // Turn On RUN_TO_POSITION
            motorfrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorfrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorbackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorbackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //used to be motorbackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            //sleep(2000);
            sleep(20);
            motorfrontLeft.setPower(Math.abs(speed));
            motorbackLeft.setPower(Math.abs(speed));
            motorfrontRight.setPower(Math.abs(speed));
            motorbackRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    //(runtime.seconds() < timeoutS) &&
                    (motorfrontLeft.isBusy() && motorfrontRight.isBusy() &&
                            motorbackLeft.isBusy() && motorbackRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d",
                        newfrontLeftTarget,  newfrontRightTarget,
                        newbackLeftTarget, newbackRightTarget);

                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                        motorfrontLeft.getCurrentPosition(),
                        motorfrontRight.getCurrentPosition(),
                        motorbackLeft.getCurrentPosition(),
                        motorbackRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorfrontLeft.setPower(0);
            motorfrontRight.setPower(0);
            motorbackLeft.setPower(0);
            motorbackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorfrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorfrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorbackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorbackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(5);   // delete if code not meant to pause
        }
    }*/

    //Strafes left & right
    /*public void encoderStrafe(double speed,
                              double fLbRInches, double fRbLInches
            ) {
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfrontLeftTarget = motorfrontLeft.getCurrentPosition() + (int)(fLbRInches * COUNTS_PER_INCH);
            newfrontRightTarget = motorfrontRight.getCurrentPosition() + (int)(fRbLInches * COUNTS_PER_INCH);
            newbackLeftTarget = motorbackLeft.getCurrentPosition() + (int)(fRbLInches * COUNTS_PER_INCH);
            newbackRightTarget = motorbackRight.getCurrentPosition() + (int)(fLbRInches * COUNTS_PER_INCH);
            motorfrontLeft.setTargetPosition(newfrontLeftTarget);
            motorfrontRight.setTargetPosition(newfrontRightTarget);
            motorbackLeft.setTargetPosition(newbackLeftTarget);
            motorbackRight.setTargetPosition(newbackRightTarget);

            // Turn On RUN_TO_POSITION
            motorfrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorfrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorbackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorbackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            sleep(20);
            motorfrontLeft.setPower(Math.abs(speed));
            motorbackLeft.setPower(Math.abs(speed));
            motorfrontRight.setPower(Math.abs(speed));
            motorbackRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    //(runtime.seconds() < timeoutS) &&
                    (motorfrontLeft.isBusy() && motorfrontRight.isBusy() &&
                            motorbackLeft.isBusy() && motorbackRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d",
                        newfrontLeftTarget,  newfrontRightTarget,
                        newbackLeftTarget, newbackRightTarget);

                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                        motorfrontLeft.getCurrentPosition(),
                        motorfrontRight.getCurrentPosition(),
                        motorbackLeft.getCurrentPosition(),
                        motorbackRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorfrontLeft.setPower(0);
            motorfrontRight.setPower(0);
            motorbackLeft.setPower(0);
            motorbackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorfrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorfrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorbackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorbackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(250);   // delete if code not meant to pause
        }
    }*/

    /* Initialize encoders */
    public void initEncoder() {
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        motorfrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorfrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorfrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorfrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorbackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorbackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d",
                motorfrontLeft.getCurrentPosition(),
                motorfrontRight.getCurrentPosition(),
                motorbackLeft.getCurrentPosition(),
                motorbackRight.getCurrentPosition());
        telemetry.update();
    }

    /** CAMERA */



    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Camera");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    /*private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }*/
}