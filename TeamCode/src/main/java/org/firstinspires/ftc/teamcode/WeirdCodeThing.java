package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import java.util.*;
import java.lang.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//import com.qualcomm.robotcore.hardware.DigitalChannel;

@Disabled
@TeleOp
public class WeirdCodeThing extends LinearOpMode {

    private static final double INCHES_PER_REV = 1.978956002259843;
    private static final double COUNTS_PER_MOTOR_REV    = 537.6;

    double leftInches;
    double rightInches;

    boolean holdUp = false;

    //This is where we set all of our variables so we can call them in future code
    double tgtPower = 0;
    int armPosition = 0;
    boolean faxmachine = false;
    //declare Drive Motors
    private DcMotor motorfrontLeft;
    private DcMotor motorfrontRight;
    private DcMotor motorbackLeft;
    private DcMotor motorbackRight;

    //Declare Mechanism Motors
    private DcMotor leftSlide;
    private DcMotor rightSlide;

    //Declare CR Servos
    private CRServo leftIntake;
    private CRServo rightIntake;

    //Declare Regular Servos
    private Servo leftForebar;
    private Servo rightForebar;

    //Time Variable
    private ElapsedTime runtime = new ElapsedTime();

    //motor speed variables
    double LF;
    double RF;
    double LB;
    double RB;
    //double TL;
    //double TR;
    //POWER
    double pwr = 3;

    //Joystick position variables
    double X1;
    double Y1;
    double X2;
    double Y2;
    double iX1;
    double iY1;
    double iX2;
    double iY2;
    //double D1;
    //double D2;

    //acceleration thingy
    double acc = 0;
    //analog values
    double joyScale = 1;
    double joyScale2 = 0.6;
    double motorMax = 0.9;

    @Override
    public void runOpMode() throws InterruptedException {



        double pos = 0;

        // Initialize Drive Motors
        motorfrontLeft = hardwareMap.dcMotor.get("motorfrontLeft");
        motorfrontRight = hardwareMap.dcMotor.get("motorfrontRight");
        motorbackLeft = hardwareMap.dcMotor.get("motorbackLeft");
        motorbackRight = hardwareMap.dcMotor.get("motorbackRight");

        //Initialize Mechanism Motors
        leftSlide = hardwareMap.dcMotor.get("leftSlide");
        rightSlide = hardwareMap.dcMotor.get("rightSlide");

        //Initialize CR (Continuous Rotation) Servos
        leftIntake = hardwareMap.crservo.get("leftIntake");
        rightIntake = hardwareMap.crservo.get("rightIntake");

        //Initialize Regular Servos
        leftForebar = hardwareMap.servo.get("leftForebar");
        rightForebar = hardwareMap.servo.get("rightForebar");

        //Initialize Drive Motors' Directions
        motorfrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorfrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorbackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorbackRight.setDirection(DcMotor.Direction.REVERSE);

        //Voltage Sensor
        /*VoltageSensor voltSensor = hardwareMap.voltageSensor.get("Motor Controller 1");
        telemetry.addLine(String.valueOf(voltSensor.getVoltage()));
        telemetry.update();*/

        //Initialize Mechanism Motors' Directions
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        //Initialize Servos' Directions
        leftForebar.setDirection(Servo.Direction.REVERSE);
        rightForebar.setDirection(Servo.Direction.FORWARD);

        initEncoder();

        telemetry.addData(">", "DRAGON: PREPARE FOR TAKEOFF");
        telemetry.update();
        //Wait to start code
        waitForStart();

        // Repeatedly run code in here until stop button is pressed
        //All motors are backwards except ones that are used
        while (opModeIsActive()) {

            if(pos==0) {
                leftForebar.setPosition(0.25);
                rightForebar.setPosition(0.25);

                /*leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/

                pos = 1;
            }

            telemetry.addData("Left Slide Position (TPR): ", leftSlide.getCurrentPosition());
            telemetry.addData("Right Slide Position (TPR): ", rightSlide.getCurrentPosition());

            leftInches = ((1)/((leftSlide.getCurrentPosition())/(COUNTS_PER_MOTOR_REV)))*INCHES_PER_REV;
            rightInches = ((1)/((rightSlide.getCurrentPosition())/(COUNTS_PER_MOTOR_REV)))*INCHES_PER_REV;

            telemetry.addData("Left Slide Position (INCHES): ", leftInches);
            telemetry.addData("Right Slide Position (INCHES): ", rightInches);

            telemetry.update();


            //reset speed variables
            LF = 0;
            RF = 0;
            LB = 0;
            RB = 0;

            if (gamepad1.right_trigger > 0.5) {
                faxmachine = !faxmachine;
            }

            iY1 = 0.7 * (Math.pow(gamepad1.right_stick_y * joyScale, pwr));
            iX1 = 0.7 * (Math.pow(gamepad1.right_stick_x * joyScale, pwr));
            iY2 = 0.7 * (Math.pow(gamepad1.left_stick_y * joyScale, pwr));
            iX2 = 0.7 * (Math.pow(gamepad1.left_stick_x * joyScale, pwr));

            if (faxmachine) {
                iY1 = gamepad1.right_stick_y * joyScale;
                iX1 = gamepad1.right_stick_x * joyScale;
                iY2 = gamepad1.left_stick_y * joyScale;
                iX2 = gamepad1.left_stick_x * joyScale;
            }

            //get joystick values
            Y1 = iY1;
            X1 = iX1;
            Y2 = iY2;
            X2 = iX2;


            telemetry.addLine(String.valueOf(gamepad1.right_stick_y));
            telemetry.addLine(String.valueOf(gamepad1.right_stick_x));
            telemetry.addLine(String.valueOf(gamepad1.left_stick_y));
            telemetry.addLine(String.valueOf(gamepad1.left_stick_x));

            telemetry.update();
/*
            if((gamepad1.right_stick_y > 0.75)||(gamepad1.right_stick_y < -0.75)) {
                Y1 = gamepad1.right_stick_y * joyScale;
            }
            else {
                Y1 = gamepad1.right_stick_y * joyScale2;
            }
            if((gamepad1.right_stick_x > 0.75)||(gamepad1.right_stick_x < -0.75)) {
                X1 = gamepad1.right_stick_x * joyScale;
            }
            else {
                X1 = gamepad1.right_stick_x * joyScale2;
            }
            if((gamepad1.left_stick_y > 0.75)||(gamepad1.left_stick_y < -0.75)) {
                Y2 = gamepad1.left_stick_y * joyScale;
            }
            else {
                Y2 = gamepad1.left_stick_y * joyScale2;
            }
            if((gamepad1.left_stick_x > 0.75)||(gamepad1.left_stick_x < -0.75)) {
                X2 = gamepad1.left_stick_x * joyScale;
            }
            else {
                X2 = gamepad1.left_stick_x * joyScale2;
            }

 */
            /*X1 = gamepad1.right_stick_x * joyScale2;
            Y2 = gamepad1.left_stick_y * joyScale2;
            X2 = gamepad1.left_stick_x * joyScale2;*/

            /*D1 = Math.pow(gamepad1.right_trigger, joyScale2);
            D2 = Math.pow(gamepad1.left_trigger, joyScale2);*/


            //Forward/Backward
            LF += Y2;
            RF += Y2;
            LB += Y2;
            RB += Y2;

            //Strafing
            LF += X2;
            RF -= X2;
            LB -= X2;
            RB += X2;

            //Turning
            LF -= X1;
            RF += X1;
            LB -= X1;
            RB += X1;

            //Wheel power limiter
            /*LF = Math.max(-motorMax, Math.min(LF, motorMax));
            RF = Math.max(-motorMax, Math.min(RF, motorMax));
            LB = Math.max(-motorMax, Math.min(LB, motorMax));
            RB = Math.max(-motorMax, Math.min(RB, motorMax));*/


            //Set Motors
            motorfrontLeft.setPower(LF);
            motorfrontRight.setPower(RF);
            motorbackLeft.setPower(LB);
            motorbackRight.setPower(RB);

            if(gamepad2.right_bumper) {
                leftIntake.setPower(-1);
                rightIntake.setPower(1);
            }
            else if(gamepad2.left_bumper) {
                leftIntake.setPower(1);
                rightIntake.setPower(-1);
            }
            else if(gamepad2.right_trigger > 0.2) {
                leftIntake.setPower(0);
                rightIntake.setPower(0);
            }
            else if(gamepad2.left_trigger > 0.2) {
                leftIntake.setPower(0);
                rightIntake.setPower(0);
            }

            if (gamepad2.left_stick_y < 0.0) {
                holdUp = false;
                leftSlide.setMode(RunMode.RUN_WITHOUT_ENCODER);
                rightSlide.setMode(RunMode.RUN_WITHOUT_ENCODER);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            }
            else if (gamepad2.left_stick_y > 0.0) {
                holdUp = false;
                leftSlide.setMode(RunMode.RUN_WITHOUT_ENCODER);
                rightSlide.setMode(RunMode.RUN_WITHOUT_ENCODER);
                leftSlide.setPower(-1);
                rightSlide.setPower(-1);
            }
            else if(gamepad2.a) {
                holdUp = false;
                leftSlide.setMode(RunMode.RUN_WITHOUT_ENCODER);
                rightSlide.setMode(RunMode.RUN_WITHOUT_ENCODER);
                leftSlide.setPower(0.1);
                rightSlide.setPower(0.1);
            }
            else if(gamepad2.y) {
                holdUp = false;
                leftSlide.setMode(RunMode.RUN_WITHOUT_ENCODER);
                rightSlide.setMode(RunMode.RUN_WITHOUT_ENCODER);
                leftSlide.setPower(-0.1);
                rightSlide.setPower(-0.1);
            }
            else if(gamepad2.dpad_up) {
                while((leftSlide.getCurrentPosition()<3700)&&(rightSlide.getCurrentPosition()<3700)) {
                    moveArm(0.2, 3700);
                    holdUp = true;
                }
            }
            else if(gamepad2.dpad_right) {
                //while((leftSlide.getCurrentPosition()<1000)&&(rightSlide.getCurrentPosition()<1000)) {
                moveArm(0.2, 1000);
                leftSlide.setMode(RunMode.RUN_WITHOUT_ENCODER);
                rightSlide.setMode(RunMode.RUN_WITHOUT_ENCODER);
                leftSlide.setPower(0);
                rightSlide.setPower(0);
                holdUp = true;
                //}
            }
            else if(gamepad2.dpad_down) {
                while((leftSlide.getCurrentPosition()<0)&&(rightSlide.getCurrentPosition()<0)) {
                    moveArm(0.05, 0);
                    holdUp = true;
                }
            }
            else if(holdUp) {
                leftSlide.setMode(RunMode.RUN_WITHOUT_ENCODER);
                rightSlide.setMode(RunMode.RUN_WITHOUT_ENCODER);
                leftSlide.setPower(0.1);
                rightSlide.setPower(0.1);
            }
            else {
                leftSlide.setMode(RunMode.RUN_WITHOUT_ENCODER);
                rightSlide.setMode(RunMode.RUN_WITHOUT_ENCODER);
                leftSlide.setPower(0);
                rightSlide.setPower(0);
            }

            if (gamepad2.x) {
                leftForebar.setPosition(0.25);
                rightForebar.setPosition(0.25);
            }
            else if (gamepad2.b) {
                leftForebar.setPosition(0.89);
                rightForebar.setPosition(0.89);
            }

        }

    }



    public void moveArm(double speed, int pos) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = pos - leftSlide.getCurrentPosition();
            newRightTarget = pos - rightSlide.getCurrentPosition();
            leftSlide.setTargetPosition(newLeftTarget);
            rightSlide.setTargetPosition(newRightTarget);



            //used to be motorbackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //sleep(2000);

            runToPosition();

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
                /*telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d",
                        newfrontLeftTarget,  newfrontRightTarget,
                        newbackLeftTarget, newbackRightTarget);

                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                        motorfrontLeft.getCurrentPosition(),
                        motorfrontRight.getCurrentPosition(),
                        motorbackLeft.getCurrentPosition(),
                        motorbackRight.getCurrentPosition());
                telemetry.update();*/
            }




            // Stop all motion;
            leftSlide.setPower(0);
            rightSlide.setPower(0);

            // Turn off RUN_TO_POSITION
            leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(5);   // delete if code not meant to pause
        }
    }

    public void runToPosition() {
        // Turn On RUN_TO_POSITION
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void initEncoder() {
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
