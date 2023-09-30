//NO

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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

//@Disabled
@TeleOp
public class KidFriendlyCode extends LinearOpMode {

    int beginning = 0;
    double linkagePosition = 0;
    double fourbarPosition = 0;
    boolean forebarIntake = false;
    boolean clawOpen = false;

    double FOREBAR_INTAKE = 0.065;
    double FOREBAR_OUTTAKE = 0.68;

    //declare Drive Motors
    private DcMotor motorfrontLeft;
    private DcMotor motorfrontRight;
    private DcMotor motorbackLeft;
    private DcMotor motorbackRight;

    //Declare Mechanism Motors
    private DcMotor leftSlide;
    private DcMotor rightSlide;

    //Declare CR Servos
    private CRServo intake;
    //Odometry Servos
    private CRServo leftOdo;
    private CRServo rightOdo;
    private CRServo frontOdo;

    //Declare Regular Servos
    private Servo leftForebar;
    private Servo rightForebar;
    private Servo rightLinkage;
    private Servo leftLinkage;
    private Servo left4bar;
    private Servo right4bar;
    private Servo claw;

    //Time Variable
    private ElapsedTime runtime = new ElapsedTime();

    //motor speed variables
    double LF;
    double RF;
    double LB;
    double RB;

    //Joystick position variables
    double X1;
    double Y1;
    double X2;
    double Y2;

    //analog values
    double joyScale = 0.6;
    double joyScale2 = 0.6;

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
        intake = hardwareMap.crservo.get("intake");

        //Odometry Servos
        leftOdo = hardwareMap.crservo.get("leftOdo");
        rightOdo = hardwareMap.crservo.get("rightOdo");
        frontOdo = hardwareMap.crservo.get("frontOdo");

        //Initialize Regular Servos
        leftForebar = hardwareMap.servo.get("leftForebar");
        rightForebar = hardwareMap.servo.get("rightForebar");
        leftLinkage =  hardwareMap.servo.get("leftLinkage");
        rightLinkage = hardwareMap.servo. get("rightLinkage");
        left4bar =  hardwareMap.servo.get("left4bar");
        right4bar = hardwareMap.servo. get("right4bar");
        claw = hardwareMap.servo. get("claw");

        //CHECK THESE

        //Initialize Drive Motors' Directions
        motorfrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorfrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorbackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorbackRight.setDirection(DcMotor.Direction.REVERSE);

        //Initialize Mechanism Motors' Directions
        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        //Initialize Servos' Directions
        leftLinkage.setDirection(Servo.Direction.REVERSE);
        rightLinkage.setDirection(Servo.Direction.FORWARD);

        leftForebar.setDirection(Servo.Direction.REVERSE);
        rightForebar.setDirection(Servo.Direction.FORWARD);

        left4bar.setDirection(Servo.Direction.REVERSE);
        right4bar.setDirection(Servo.Direction.FORWARD);

        initEncoder();

        telemetry.addData("DRAGON: ", "I am in extremely extreme agony");
        telemetry.update();

        //Wait to start code
        waitForStart();

        // Repeatedly run code in here until stop button is pressed
        //All motors are backwards except ones that are used
        while (opModeIsActive()) {
            if (beginning==0) {
                leftLinkage.setPosition(0);
                rightLinkage.setPosition(0);
                linkagePosition = 0;

                beginning++;
            }

            //reset speed variables
            LF = 0;
            RF = 0;
            LB = 0;
            RB = 0;

            Y1 = gamepad1.right_stick_y * joyScale;
            X1 = gamepad1.right_stick_x * joyScale;
            Y2 = gamepad1.left_stick_y * joyScale;
            X2 = gamepad1.left_stick_x * joyScale2;


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


            //Set Motors
            motorfrontLeft.setPower(LF);
            motorfrontRight.setPower(RF);
            motorbackLeft.setPower(LB);
            motorbackRight.setPower(RB);


            if (gamepad1.y) {
                liftOdo(4.0);
                stopOdo(0.1);
            }

            if(gamepad2.right_stick_y < 0) {
                linkagePosition = linkagePosition + 0.001;

                if(linkagePosition > 0.4) {
                    leftLinkage.setPosition(0.4);
                    rightLinkage.setPosition(0.4);

                    linkagePosition = 0.4;
                }
                else {
                    leftLinkage.setPosition(linkagePosition);
                    rightLinkage.setPosition(linkagePosition);
                }

            }
            else if(gamepad2.right_stick_y > 0) {
                linkagePosition = linkagePosition - 0.001;

                if(fourbarPosition < 0) {
                    leftLinkage.setPosition(0);
                    rightLinkage.setPosition(0);

                    linkagePosition = 0;
                }
                else {
                    leftLinkage.setPosition(linkagePosition);
                    rightLinkage.setPosition(linkagePosition);
                }

            }

            if(gamepad2.dpad_down) {
                fourbarPosition = fourbarPosition + 0.001;

                if(fourbarPosition > 0.79) {
                    left4bar.setPosition(0.79);
                    right4bar.setPosition(0.79);

                    fourbarPosition = 0.79;
                }
                else {
                    left4bar.setPosition(fourbarPosition);
                    right4bar.setPosition(fourbarPosition);
                }

            }
            else if(gamepad2.dpad_up) {
                fourbarPosition = fourbarPosition - 0.001;

                if(fourbarPosition < 0) {
                    left4bar.setPosition(0);
                    right4bar.setPosition(0);

                    fourbarPosition = 0;
                }
                else {
                    left4bar.setPosition(fourbarPosition);
                    right4bar.setPosition(fourbarPosition);
                }

            }


            if (gamepad2.right_bumper) {
                intake.setPower(-1);
            }
            else if (gamepad2.left_bumper) {
                intake.setPower(1);
            }
            else if (gamepad2.left_trigger > 0.2) {
                intake.setPower(0);
            }
            else if (gamepad2.right_trigger > 0.2) {
                intake.setPower(0);
            }


            if (gamepad2.left_stick_y < 0.0) {
                leftSlide.setPower(-0.5*gamepad2.left_stick_y);
                rightSlide.setPower(-0.5*gamepad2.left_stick_y);
            }
            else if (gamepad2.left_stick_y > 0.0) {
                leftSlide.setPower(-0.5*gamepad2.left_stick_y);
                rightSlide.setPower(-0.5*gamepad2.left_stick_y);
            }
            else {
                leftSlide.setPower(0.1);
                rightSlide.setPower(0.1);
            }

            if (gamepad2.x) {
                if(forebarIntake) {
                    keepForebarO(1.0);
                    forebarIntake = false;
                }
                else if(!forebarIntake) {
                    keepForebarI(1.0);
                    forebarIntake = true;
                }
            }

            if (gamepad2.y) {
                if(clawOpen) {
                    keepClaw(1.0, 0.6);
                    clawOpen = false;
                }
                else if(!clawOpen) {
                    keepClaw(1.0, 0);
                    clawOpen = true;
                }
            }
        }
    }

    public void keepForebarO(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            leftForebar.setPosition(FOREBAR_OUTTAKE);
            rightForebar.setPosition(FOREBAR_OUTTAKE + 0.065);
            forebarIntake = false;
        }
    }

    public void keepClaw(double time, double pos) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            claw.setPosition(pos);
            clawOpen = false;
        }
    }

    public void keepForebarI(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            leftForebar.setPosition(FOREBAR_INTAKE);
            rightForebar.setPosition(FOREBAR_INTAKE + 0.065);
            forebarIntake = true;
        }
    }

    public void liftOdo(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run) {
            leftOdo.setPower(-1.0);
            rightOdo.setPower(-0.6);
            frontOdo.setPower(-1.0);
        }
    }

    public void stopOdo(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            leftOdo.setPower(0);
            rightOdo.setPower(0);
            frontOdo.setPower(0);
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