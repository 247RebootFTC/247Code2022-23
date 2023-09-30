//Encoders

package org.firstinspires.ftc.teamcode;

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

@Autonomous

public class BasicEncoderAutonomous extends LinearOpMode {

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


        //Initialize drive motors' direction
        motorfrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorfrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorbackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorbackRight.setDirection(DcMotor.Direction.FORWARD);

        // Initialize encoders
        initEncoder();

        //encoderDrive(DRIVE_SPEED, 36, 36);
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