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

import com.qualcomm.robotcore.hardware.DigitalChannel;

@Disabled
@TeleOp
public class BestCode extends LinearOpMode {

    private static final double INCHES_PER_REV = 1.978956002259843;
    private static final double COUNTS_PER_MOTOR_REV    = 537.6;

    double leftInches;
    double rightInches;

    boolean holdUp = false;
    boolean moveUp = false;
    boolean moveDown = false;
    double pos = 0;

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
    private CRServo frontIntake;
    private CRServo backIntake;

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

    //analog values
    double joyScale = 0.8;
    double joyScale2 = 0.6;
    double motorMax = 0.9;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize Drive Motors
        motorfrontLeft = hardwareMap.dcMotor.get("motorfrontLeft");
        motorfrontRight = hardwareMap.dcMotor.get("motorfrontRight");
        motorbackLeft = hardwareMap.dcMotor.get("motorbackLeft");
        motorbackRight = hardwareMap.dcMotor.get("motorbackRight");

        //Initialize Mechanism Motors
        leftSlide = hardwareMap.dcMotor.get("leftSlide");
        rightSlide = hardwareMap.dcMotor.get("rightSlide");

        //Initialize CR (Continuous Rotation) Servos
        frontIntake = hardwareMap.crservo.get("frontIntake");
        backIntake = hardwareMap.crservo.get("backIntake");

        //Initialize Regular Servos
        leftForebar = hardwareMap.servo.get("leftForebar");
        rightForebar = hardwareMap.servo.get("rightForebar");

        //Initialize Drive Motors' Directions
        motorfrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorfrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorbackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorbackRight.setDirection(DcMotor.Direction.REVERSE);

        //Initialize Mechanism Motors' Directions
        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        //Initialize Servos' Directions
        leftForebar.setDirection(Servo.Direction.FORWARD);
        rightForebar.setDirection(Servo.Direction.REVERSE);

        initEncoder();

        telemetry.addData("DRAGON: ", "I am in extreme agony");
        telemetry.update();

        //Wait to start code
        waitForStart();

        // Repeatedly run code in here until stop button is pressed
        //All motors are backwards except ones that are used
        while (opModeIsActive()) {

            //automatically sets fore
            if (pos == 0) {
                leftForebar.setPosition(0.25);
                rightForebar.setPosition(0.25);

                pos = 1;
            }

            telemetry.addLine(String.valueOf(leftSlide.getCurrentPosition()));
            telemetry.addLine(String.valueOf(rightSlide.getCurrentPosition()));

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

            if (gamepad2.right_bumper) {
                frontIntake.setPower(-1);
                backIntake.setPower(1);
            } else if (gamepad2.left_bumper) {
                frontIntake.setPower(1);
                backIntake.setPower(-1);
            } else if (gamepad2.right_trigger > 0.2) {
                frontIntake.setPower(0);
                backIntake.setPower(0);
            } else if (gamepad2.left_trigger > 0.2) {
                frontIntake.setPower(0);
                backIntake.setPower(0);
            }


            if(moveUp) {
                if(((-1*leftSlide.getCurrentPosition()) >= pos) || ((-1*rightSlide.getCurrentPosition()) >= pos)) {
                    moveUp = false;
                    holdUp = true;
                    leftSlide.setPower(0.1);
                    rightSlide.setPower(0.1);
                    telemetry.addLine("MOVE UP FALSE");
                    telemetry.update();
                }
                else {
                    leftSlide.setPower(1.0);
                    rightSlide.setPower(1.0);
                    telemetry.addLine("MOVE UP TRUE");
                    telemetry.update();
                }
            }
            else if(moveDown) {
                if(((-1*leftSlide.getCurrentPosition()) <= pos) || ((-1*rightSlide.getCurrentPosition()) <= pos)) {
                    moveDown = false;
                    holdUp = true;
                    leftSlide.setPower(0.1);
                    rightSlide.setPower(0.1);
                    telemetry.addLine("MOVE DOWN FALSE");
                    telemetry.update();
                }
                else {
                    leftSlide.setPower(-1.0);
                    rightSlide.setPower(-1.0);
                    telemetry.addLine("MOVE DOWN TRUE");
                    telemetry.update();
                }
            }
            else if (gamepad2.left_stick_y < 0.0) {
                holdUp = false;
                moveUp = false;
                moveDown = false;
                leftSlide.setPower(0.6);
                rightSlide.setPower(0.6);
            }
            else if (gamepad2.left_stick_y > 0.0) {
                holdUp = false;
                moveUp = false;
                moveDown = false;
                leftSlide.setPower(-0.6);
                rightSlide.setPower(-0.6);
            }
            else if(gamepad2.a) {
                holdUp = false;
                moveUp = false;
                moveDown = false;
                leftSlide.setPower(0.1);
                rightSlide.setPower(0.1);
            }
            else if(gamepad2.y) {
                holdUp = false;
                moveUp = false;
                moveDown = false;
                leftSlide.setPower(-0.1);
                rightSlide.setPower(-0.1);
            }
            else if(gamepad2.dpad_up) {
                //High Junction
                holdUp = false;
                moveUp = false;
                moveDown = false;
                pos = 2900;
                moveArm();
            }
            else if(gamepad2.dpad_right) {
                //Middle Junction
                holdUp = false;
                moveUp = false;
                moveDown = false;
                pos = 1200;
                moveArm();
            }
            else if(gamepad2.dpad_down) {
                //Low Junction
                holdUp = false;
                moveUp = false;
                moveDown = false;
                pos = 700;
                moveArm();
            }
            else if(gamepad2.dpad_left) {
                telemetry.addLine("IN DPAD LEFT IF STATEMENT");
                telemetry.update();
                //Intake Height
                holdUp = false;
                moveUp = false;
                moveDown = false;
                pos = 300;
                moveArm();
            }
            else if(holdUp) {
                moveUp = false;
                moveDown = false;
                leftSlide.setPower(0.1);
                rightSlide.setPower(0.1);
                telemetry.addLine("HOLD UP FUNCTION");
                telemetry.update();
            }
            else {
                moveUp = false;
                moveDown = false;
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

    public void moveArm() {

        telemetry.addLine("LEFT SLIDE POS: " + String.valueOf(leftSlide.getCurrentPosition()));
        telemetry.addLine("RIGHT SLIDE POS: " + String.valueOf(rightSlide.getCurrentPosition()));
        telemetry.addLine("POS: " + String.valueOf(pos));
        telemetry.update();

        //MOVE UP
        if((pos > (-1*leftSlide.getCurrentPosition()))&&((pos > (-1*rightSlide.getCurrentPosition())))) {

            telemetry.addLine("IN MOVE UP STATEMENT");
            telemetry.update();

            leftSlide.setPower(0.75);
            rightSlide.setPower(0.75);

            moveUp = true;
        }
        //MOVE DOWN
        else if((pos < (-1*leftSlide.getCurrentPosition()))&&((pos < (-1*rightSlide.getCurrentPosition())))) {

            telemetry.addLine("IN MOVE DOWN STATEMENT");
            telemetry.update();

            leftSlide.setPower(-0.2);
            rightSlide.setPower(-0.2);

            moveDown = true;
        }

    }

    public void initEncoder() {
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        /*telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d",
                leftSlide.getCurrentPosition(),
                rightSlide.getCurrentPosition());
        telemetry.update();*/
    }

}