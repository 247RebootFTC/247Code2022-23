//NO

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
import java.util.*;
import java.lang.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//import com.qualcomm.robotcore.hardware.DigitalChannel;

@Disabled
@TeleOp (name="Reuben's Wild Practice")
public class ReubensWildPractice extends LinearOpMode {

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
    //private DcMotor leftSlide;
    //private DcMotor rightSlide;

    //Declare CR Servos
    //private CRServo frontIntake;
    //private CRServo backIntake;

    //Declare Regular Servos
    //private Servo leftForebar;
    //private Servo rightForebar;

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
        //leftSlide = hardwareMap.dcMotor.get("leftSlide");
        //rightSlide = hardwareMap.dcMotor.get("rightSlide");

        //Initialize CR (Continuous Rotation) Servos
        //frontIntake = hardwareMap.crservo.get("frontIntake");
        //backIntake = hardwareMap.crservo.get("backIntake");

        //Initialize Regular Servos
        //leftForebar = hardwareMap.servo.get("leftForebar");
        //rightForebar = hardwareMap.servo.get("rightForebar");

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
        //leftSlide.setDirection(DcMotor.Direction.REVERSE);
        //rightSlide.setDirection(DcMotor.Direction.REVERSE);

        //Initialize Servos' Directions
        //leftForebar.setDirection(Servo.Direction.REVERSE);
        //rightForebar.setDirection(Servo.Direction.FORWARD);

        //Wait to start code
        waitForStart();

        // Repeatedly run code in here until stop button is pressed
        //All motors are backwards except ones that are used
        while (opModeIsActive()) {

            if (pos == 0) {
                //leftForebar.setPosition(0.25);
                //rightForebar.setPosition(0.25);
                pos = 1;
            }


            //reset speed variables
            LF = 0;
            RF = 0;
            LB = 0;
            RB = 0;

            if (gamepad1.right_bumper) {
                faxmachine = !faxmachine;
            }
            iY1 = (Math.pow(gamepad1.right_stick_y * joyScale, pwr));
            iX1 = (Math.pow(gamepad1.right_stick_x * joyScale, pwr));
            iY2 = (Math.pow(gamepad1.left_stick_y * joyScale, pwr));
            iX2 = (Math.pow(gamepad1.left_stick_x * joyScale, pwr));

            if (faxmachine) {
                iY1 = 0.5*gamepad1.right_stick_y * joyScale;
                iX1 = 0.5*gamepad1.right_stick_x * joyScale;
                iY2 = 0.5*gamepad1.left_stick_y * joyScale;
                iX2 = 0.5*gamepad1.left_stick_x * joyScale;
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


            //Foward/Backward
            LF -= Y2;
            RF -= Y2;
            LB -= Y2;
            RB -= Y2;

            //Strafing
            /*
            LF += X2;
            RF -= X2;
            LB -= X2;
            RB += X2;
            */
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
            if(gamepad1.dpad_right)
            {
                motorfrontLeft.setPower(0.80);
                motorfrontRight.setPower(-0.80);
                motorbackRight.setPower(0.80);
                motorbackLeft.setPower(-0.80);
            }
            if(gamepad1.dpad_left)
            {
                motorfrontLeft.setPower(-0.80);
                motorfrontRight.setPower(0.80);
                motorbackRight.setPower(-0.80);
                motorbackLeft.setPower(0.80);
            }
            /*
            if(gamepad2.right_bumper) {
                frontIntake.setPower(-1);
                backIntake.setPower(1);
            }
            else if(gamepad2.left_bumper) {
                frontIntake.setPower(1);
                backIntake.setPower(-1);
            }
            else if(gamepad2.right_trigger > 0.2) {
                frontIntake.setPower(0);
                backIntake.setPower(0);
            }

            else if(gamepad2.left_trigger > 0.2) {
                frontIntake.setPower(0);
                backIntake.setPower(0);
            }

            if (gamepad2.left_stick_y < 0.0) {
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            }
            else if (gamepad2.left_stick_y > 0.0) {
                leftSlide.setPower(-1);
                rightSlide.setPower(-1);
            }
            else {
                leftSlide.setPower(0);
                rightSlide.setPower(0);
            }
            if (gamepad2.x) {
                leftForebar.setPosition(0.25);
                rightForebar.setPosition(0.25);
            }
            else if (gamepad2.b) {
                leftForebar.setPosition(1.0);
                rightForebar.setPosition(1.0);
            }
        }
        */
        /*if(gamepad1.y) {
            VoltageSensor voltSensor = hardwareMap.voltageSensor.get("Motor Controller 1");
            double voltage = 15/voltSensor.getVoltage();
            if(armPosition==0) {
                //armUp(voltage*0.75);
            }
            else if(armPosition==1) {
                //armUp(voltage*0.5);
            }
            if(armPosition==2) {
                //armUp(voltage*0.25);
            }
            armPosition = 3;
        }
        if(gamepad1.b) {
            VoltageSensor voltSensor = hardwareMap.voltageSensor.get("Motor Controller 1");
            double voltage = 15/voltSensor.getVoltage();
            if(armPosition==0) {
                //armUp(voltage*0.5);
            }
            else if(armPosition==1) {
                //armUp(voltage*0.25);
            }
            if(armPosition==3) {
                //armDown(voltage*0.25);
            }
            armPosition = 2;
        }
        if(gamepad1.a) {
            VoltageSensor voltSensor = hardwareMap.voltageSensor.get("Motor Controller 1");
            double voltage = 15/voltSensor.getVoltage();
            if(armPosition==0) {
                //armUp(voltage*0.25);
            }
            else if(armPosition==2) {
                //armDown(voltage*0.25);
            }
            if(armPosition==3) {
                //armDown(voltage*0.5);
            }
            armPosition = 1;
        }
        if(gamepad1.x) {
            VoltageSensor voltSensor = hardwareMap.voltageSensor.get("Motor Controller 1");
            double voltage = 15/voltSensor.getVoltage();
            if(armPosition==1) {
                //armDown(voltage*0.25);
            }
            else if(armPosition==2) {
                //armDown(voltage*0.5);
            }
            if(armPosition==3) {
                //armDown(voltage*0.75);
            }
            armPosition = 0;
        }*/
        }
    /*public void armUp(double time) {
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            leftSlide.setPower(1);
            rightSlide.setPower(1);
        }
    }
    public void armDown(double time) {
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            leftSlide.setPower(-1);
            rightSlide.setPower(-1);
        }
    }*/
    }
}
