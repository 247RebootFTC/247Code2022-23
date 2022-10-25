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

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import java.util.Iterator;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import java.security.Guard;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
//import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@Autonomous(name = "Park", group = "Red Autonomi")

public class Park extends LinearOpMode {

    /* Encoder Specific defines */
    private static final double COUNTS_PER_MOTOR_REV    = 675; //828.8 //537.6   // eg: TETRIX Motor Encoder
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
    private static final double DRIVE_SPEED             = 0.2;//0.4, 1.0; //this is for wheels only, make new variables for other motors
    private static final double MAX_SPEED               = 0.5;
    private static final double TURN_SPEED              = 0.4; //this is for wheels only, make new variables for other motors


    //declare Drive Motors
    private DcMotor motorfrontLeft;
    private DcMotor motorfrontRight;
    private DcMotor motorbackLeft;
    private DcMotor motorbackRight;

    //Declare Mechanism Motors
    //private DcMotor IntakeMotor;
    //private DcMotor ArmMotor;


    //Declare servos

    //the time Object
    private ElapsedTime runtime=new ElapsedTime();
    String Label = "";

    @Override
    public void runOpMode() throws InterruptedException {
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        //Initialize drive motors
        motorfrontLeft = hardwareMap.dcMotor.get("motorfrontLeft");
        motorfrontRight = hardwareMap.dcMotor.get("motorfrontRight");
        motorbackLeft = hardwareMap.dcMotor.get("motorbackLeft");
        motorbackRight = hardwareMap.dcMotor.get("motorbackRight");

        //Initialize other motors
        //IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
        //ArmMotor = hardwareMap.dcMotor.get("ArmMotor");

        //Initialize servos


        //Initialize drive motors' direction
        //DONT CHANGE THIS CONFIRGURATION
        motorfrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorfrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorbackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorbackRight.setDirection(DcMotor.Direction.FORWARD);

        // Initialize encoders
        initEncoder();

        //encoderDrive(DRIVE_SPEED, 36, 36);
        moveForward(0.5);
        Stop(1.0);
    }

    /** FUNCTIONS */

    /** TIME-BASED */

    //Move backwards
    public void moveBack(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(-1);
            motorfrontRight.setPower(-1);
            motorbackLeft.setPower(-1);
            motorbackRight.setPower(-1);
        }
    }

    //Move forwards
    public void moveForward(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(1);
            motorfrontRight.setPower(1);
            motorbackLeft.setPower(1);
            motorbackRight.setPower(1);
        }
    }

    //Turn right
    public void turnRight(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(1);
            motorfrontRight.setPower(-1);
            motorbackLeft.setPower(1);
            motorbackRight.setPower(-1);
        }
    }

    //Turn left
    public void turnLeft(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(-1);
            motorfrontRight.setPower(1);
            motorbackLeft.setPower(-1);
            motorbackRight.setPower(1);
        }
    }

    //Strafe right
    public void strafeRight(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(-1);
            motorfrontRight.setPower(1);
            motorbackLeft.setPower(1);
            motorbackRight.setPower(-1);
        }
    }

    //Strafe left
    public void strafeLeft(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(1);
            motorfrontRight.setPower(-1);
            motorbackLeft.setPower(-1);
            motorbackRight.setPower(1);
        }
    }
    /*public void BlueCarouselSpin(double time){
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            CarouselMotor.setPower(-1);
        }
    }
    public void RedCarouselSpin(double time){
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            CarouselMotor.setPower(1);
        }
    }*/
    public void Stop(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(0);
            motorfrontRight.setPower(0);
            motorbackLeft.setPower(0);
            motorbackRight.setPower(0);
            //CarouselMotor.setPower(0);
        }
    }

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
    public void encoderDrive(double speed,
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
    }

    //Strafes left & right
    public void encoderStrafe(double speed,
                              double fLbRInches, double fRbLInches
            /*double timeoutS*/) {
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
            sleep(2000);
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

            sleep(250);   // delete if code not meant to pause
        }
    }

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
}