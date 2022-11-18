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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Disabled
@TeleOp
public class ArmEncoderTest extends LinearOpMode {

    /* Encoder Specific defines
    private static final double COUNTS_PER_MOTOR_REV    = 537.6;   // eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION    = 1 ;     // 1 IF MOUNTED DIRECTLY TO MOTOR IF NOT-
    //-divide the number of teeth on the wheel sprocket-
    //-by the number of teeth on the motor sprocket
    //This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415); //this is for wheels only, make new variables for other motors
    //private static final double COUNTS_PER_INCH_CAL     = COUNTS_PER_INCH + 300;
    private static final double DRIVE_SPEED             = 0.4;
*/

    private static final double INCHES_PER_REV = 1.978956002259843;
    private static final double COUNTS_PER_MOTOR_REV    = 537.6;

    //Declare Drive Motors
    private DcMotor leftSlide;
    private DcMotor rightSlide;

    //The Time Object
    private ElapsedTime runtime=new ElapsedTime();

    double leftInches;
    double rightInches;

    @Override
    public void runOpMode() {

        leftSlide = hardwareMap.dcMotor.get("leftSlide");
        rightSlide = hardwareMap.dcMotor.get("rightSlide");

        //Initialize drive motors' direction
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Wait for the game to begin */
        telemetry.addData(">", "Pray That the Autonomous Works");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Left Slide Position (TPR): ", leftSlide.getCurrentPosition());
            telemetry.addData("Right Slide Position (TPR): ", rightSlide.getCurrentPosition());

            leftInches = ((1)/((leftSlide.getCurrentPosition())/(COUNTS_PER_MOTOR_REV)))*INCHES_PER_REV;
            rightInches = ((1)/((rightSlide.getCurrentPosition())/(COUNTS_PER_MOTOR_REV)))*INCHES_PER_REV;

            telemetry.addData("Left Slide Position (INCHES): ", leftInches);
            telemetry.addData("Right Slide Position (INCHES): ", rightInches);

            telemetry.update();

            if (gamepad2.left_stick_y < 0.0) {
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            }
            else if (gamepad2.left_stick_y > 0.0) {
                leftSlide.setPower(-1);
                rightSlide.setPower(-1);
            }
            else if(gamepad2.a) {
                leftSlide.setPower(0.1);
                rightSlide.setPower(0.1);
            }
            else if(gamepad2.y) {
                leftSlide.setPower(-0.1);
                rightSlide.setPower(-0.1);
            }
            else {
                leftSlide.setPower(0);
                rightSlide.setPower(0);
            }
        }
    }

    /** FUNCTIONS */


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
    /*public void encoderArm(double speed, double inches) {
        int newLeftSlideTarget;
        int newRightSlideTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftSlideTarget = leftSlide.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightSlideTarget = rightSlide.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            leftSlide.setTargetPosition(newLeftSlideTarget);
            rightSlide.setTargetPosition(newRightSlideTarget);

            // Turn On RUN_TO_POSITION
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            sleep(20);
            leftSlide.setPower(Math.abs(speed));
            rightSlide.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (leftSlide.isBusy() && rightSlide.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Target", newLeftSlideTarget, newRightSlideTarget);

                telemetry.addData("Current Position: ", leftSlide.getCurrentPosition(), rightSlide.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftSlide.setPower(0);
            rightSlide.setPower(0);

            // Turn off RUN_TO_POSITION
        }
    }*/

    /* Initialize encoders */
    public void initEncoder() {
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}