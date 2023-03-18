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
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import java.util.*;
import java.lang.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DigitalChannel;

//@Disabled
@TeleOp
public class ServoZeroer extends LinearOpMode {
    private Servo rightLinkage;
    private Servo leftLinkage;
    private Servo leftForebar;
    private Servo rightForebar;
    private Servo left4bar;
    private Servo right4bar;
    private Servo claw;
    private CRServo intake;

    private DcMotor leftSlide;
    private DcMotor rightSlide;

    //Time Variable
    private ElapsedTime runtime = new ElapsedTime();

    int pos = 0;

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
    double joyScale = 0.6;
    double joyScale2 = 0.6;
    double motorMax = 0.9;

    double linkagePos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        leftLinkage =  hardwareMap.servo.get("leftLinkage");
        rightLinkage = hardwareMap.servo. get("rightLinkage");

        leftForebar =  hardwareMap.servo.get("leftForebar");
        rightForebar = hardwareMap.servo. get("rightForebar");

        left4bar =  hardwareMap.servo.get("left4bar");
        right4bar = hardwareMap.servo. get("right4bar");

        claw = hardwareMap.servo. get("claw");
        intake = hardwareMap.crservo.get("intake");

        leftLinkage.setDirection(Servo.Direction.REVERSE);
        rightLinkage.setDirection(Servo.Direction.FORWARD);

        leftForebar.setDirection(Servo.Direction.REVERSE);
        rightForebar.setDirection(Servo.Direction.FORWARD);

        left4bar.setDirection(Servo.Direction.REVERSE);
        right4bar.setDirection(Servo.Direction.FORWARD);


        //Wait to start code
        waitForStart();

        // Repeatedly run code in here until stop button is pressed
        //All motors are backwards except ones that are used
        while (opModeIsActive()) {

            telemetry.addData("Left 4bar: ", left4bar.getPosition());
            telemetry.addData("Right 4bar: ", right4bar.getPosition());
            telemetry.update();

            //1 : .0
            //2 : .065
            //3: .1
            //4: .15
            //5: .175

            if(gamepad2.x) {//Cone 5

                left4bar.setPosition(0.45);
                right4bar.setPosition(0.45);
                stop(0.01);
                leftLinkage.setPosition(0.38);
                rightLinkage.setPosition(0.38);
                stop(0.4);
                left4bar.setPosition(0.15);
                right4bar.setPosition(0.15);
                stop(0.6);
                claw.setPosition(0.35);
                stop(0.25);
                left4bar.setPosition(0.45);
                right4bar.setPosition(0.45);
                stop(0.25);
                leftLinkage.setPosition(0);
                rightLinkage.setPosition(0);
                stop(0.8);
                left4bar.setPosition(0.65);
                right4bar.setPosition(0.65);
                stop(0.3);
                claw.setPosition(0);
                stop(0.1);

            }
            if(gamepad2.dpad_left) {//Cone 4

                left4bar.setPosition(0.45);
                right4bar.setPosition(0.45);
                stop(0.01);
                leftLinkage.setPosition(0.4);
                rightLinkage.setPosition(0.4);
                stop(0.4);
                left4bar.setPosition(0.12);
                right4bar.setPosition(0.12);
                stop(0.6);
                claw.setPosition(0.35);
                stop(0.25);
                left4bar.setPosition(0.45);
                right4bar.setPosition(0.45);
                stop(0.25);
                leftLinkage.setPosition(0);
                rightLinkage.setPosition(0);
                stop(0.8);
                left4bar.setPosition(0.65);
                right4bar.setPosition(0.65);
                stop(0.3);
                claw.setPosition(0);
                stop(0.1);
            }
            if(gamepad2.dpad_down) {//Cone 3

                left4bar.setPosition(0.45);
                right4bar.setPosition(0.45);
                stop(0.01);
                leftLinkage.setPosition(0.4);
                rightLinkage.setPosition(0.4);
                stop(0.4);
                left4bar.setPosition(0.085);
                right4bar.setPosition(0.085);
                stop(0.7);
                claw.setPosition(0.35);
                stop(0.25);
                left4bar.setPosition(0.45);
                right4bar.setPosition(0.45);
                stop(0.25);
                leftLinkage.setPosition(0);
                rightLinkage.setPosition(0);
                stop(0.8);
                left4bar.setPosition(0.65);
                right4bar.setPosition(0.65);
                stop(0.3);
                claw.setPosition(0);
                stop(0.1);
            }

            if(gamepad2.dpad_right) {//Cone 2
                left4bar.setPosition(0.45);
                right4bar.setPosition(0.45);
                stop(0.01);
                leftLinkage.setPosition(0.4);
                rightLinkage.setPosition(0.4);
                stop(0.4);
                left4bar.setPosition(0.035);
                right4bar.setPosition(0.035);
                stop(0.7);
                claw.setPosition(0.35);
                stop(0.25);
                left4bar.setPosition(0.45);
                right4bar.setPosition(0.45);
                stop(0.25);
                leftLinkage.setPosition(0);
                rightLinkage.setPosition(0);
                stop(0.8);
                left4bar.setPosition(0.65);
                right4bar.setPosition(0.65);
                stop(0.3);
                claw.setPosition(0);
                stop(0.1);
            }
            if(gamepad2.dpad_up) {//Cone 1

                left4bar.setPosition(0.45);
                right4bar.setPosition(0.45);
                stop(0.01);
                leftLinkage.setPosition(0.425);
                rightLinkage.setPosition(0.425);
                stop(0.4);
                left4bar.setPosition(0.025);
                right4bar.setPosition(0.025);
                stop(0.7);
                claw.setPosition(0.35);
                stop(0.25);
                left4bar.setPosition(0.45);
                right4bar.setPosition(0.45);
                stop(0.25);
                leftLinkage.setPosition(0);
                rightLinkage.setPosition(0);
                stop(0.8);
                left4bar.setPosition(0.65);
                right4bar.setPosition(0.65);
                stop(0.3);
                claw.setPosition(0);
                stop(0.1);
            }

            if(gamepad2.y) { //GIGA BUTTON
                left4bar.setPosition(0.45);
                right4bar.setPosition(0.45);
                stop(0.01);
                leftLinkage.setPosition(0.34);
                rightLinkage.setPosition(0.34);
                stop(0.4);
                left4bar.setPosition(0.14);
                right4bar.setPosition(0.14);
                stop(0.7);
                claw.setPosition(0.35);
                stop(0.25);
                left4bar.setPosition(0.45);
                right4bar.setPosition(0.45);
                stop(0.25);
                leftLinkage.setPosition(0);
                rightLinkage.setPosition(0);
                stop(0.8);
                left4bar.setPosition(0.7);
                right4bar.setPosition(0.7);
                stop(0.3);
                claw.setPosition(0);
                stop(0.25);

                left4bar.setPosition(0.45);
                right4bar.setPosition(0.45);
                stop(0.01);
                leftLinkage.setPosition(0.34);
                rightLinkage.setPosition(0.34);
                stop(0.4);
                left4bar.setPosition(0.12);
                right4bar.setPosition(0.12);
                stop(0.6);
                claw.setPosition(0.35);
                stop(0.25);
                left4bar.setPosition(0.45);
                right4bar.setPosition(0.45);
                stop(0.25);
                leftLinkage.setPosition(0);
                rightLinkage.setPosition(0);
                stop(0.8);
                left4bar.setPosition(0.7);
                right4bar.setPosition(0.7);
                stop(0.3);
                claw.setPosition(0);
                stop(0.25);

                left4bar.setPosition(0.45);
                right4bar.setPosition(0.45);
                stop(0.01);
                leftLinkage.setPosition(0.34);
                rightLinkage.setPosition(0.34);
                stop(0.4);
                left4bar.setPosition(0.09);
                right4bar.setPosition(0.09);
                stop(0.7);
                claw.setPosition(0.35);
                stop(0.25);
                left4bar.setPosition(0.45);
                right4bar.setPosition(0.45);
                stop(0.25);
                leftLinkage.setPosition(0);
                rightLinkage.setPosition(0);
                stop(0.8);
                left4bar.setPosition(0.7);
                right4bar.setPosition(0.7);
                stop(0.3);
                claw.setPosition(0);
                stop(0.25);

                left4bar.setPosition(0.45);
                right4bar.setPosition(0.45);
                stop(0.01);
                leftLinkage.setPosition(0.38);
                rightLinkage.setPosition(0.38);
                stop(0.4);
                left4bar.setPosition(0.04);
                right4bar.setPosition(0.04);
                stop(0.7);
                claw.setPosition(0.35);
                stop(0.25);
                left4bar.setPosition(0.25);
                right4bar.setPosition(0.25);
                stop(0.1);
                leftLinkage.setPosition(0);
                rightLinkage.setPosition(0);
                stop(0.8);
                left4bar.setPosition(0.7);
                right4bar.setPosition(0.7);
                stop(0.5);
                claw.setPosition(0);
                stop(0.25);

                left4bar.setPosition(0.45);
                right4bar.setPosition(0.45);
                stop(0.01);
                leftLinkage.setPosition(0.38);
                rightLinkage.setPosition(0.38);
                stop(0.4);
                left4bar.setPosition(0.02);
                right4bar.setPosition(0.02);
                stop(0.7);
                claw.setPosition(0.35);
                stop(0.25);
                left4bar.setPosition(0.25);
                right4bar.setPosition(0.25);
                stop(0.1);
                leftLinkage.setPosition(0);
                rightLinkage.setPosition(0);
                stop(0.8);
                left4bar.setPosition(0.7);
                right4bar.setPosition(0.7);
                stop(0.5);
                claw.setPosition(0);
                stop(0.1);
            }

            if(gamepad2.b) {
                leftLinkage.setPosition(0.40);
                rightLinkage.setPosition(0.40);
            }

            if(gamepad1.y) {//open claw
                leftLinkage.setPosition(0);
                rightLinkage.setPosition(0);
            }

            else if(gamepad1.a) {//close claw
                left4bar.setPosition(0.6);
                right4bar.setPosition(0.6);
            }

            else if(gamepad1.x) {//close claw
                left4bar.setPosition(0.165);
                right4bar.setPosition(0.165);
            }

            if(gamepad1.b) {//O button
                leftLinkage.setPosition(0.4);
                rightLinkage.setPosition(0.4);
            }

            if(gamepad1.dpad_up)  {
                left4bar.setPosition(0);
                right4bar.setPosition(0);
            }
            if(gamepad1.dpad_right)  {
                left4bar.setPosition(0.05);
                right4bar.setPosition(0.05);
            }
            if(gamepad1.dpad_down)  {
                left4bar.setPosition(0.1);
                right4bar.setPosition(0.1);
            }
            if(gamepad1.dpad_left)  {
                left4bar.setPosition(0.14);
                right4bar.setPosition(0.14);
            }
            if(gamepad1.left_bumper) {
                claw.setPosition(0);
            }
            if(gamepad1.right_bumper) {
                claw.setPosition(0.35);
            }

/*
            if(gamepad1.dpad_up) {//X button
                leftLinkage.setPosition(0.215);
                rightLinkage.setPosition(0.215);
                left4bar.setPosition(.37);
                right4bar.setPosition(.36);
                stop(2.0, .37);
                claw.setPosition(0.25);
                stop(1.0, .37);
                leftLinkage.setPosition(0.36);
                rightLinkage.setPosition(0.36);
                stop(0.05, .37);
                left4bar.setPosition(0.10);
                right4bar.setPosition(0.09);
            }
            if(gamepad1.dpad_right) {//X button
                leftLinkage.setPosition(0.21);
                rightLinkage.setPosition(0.21);
                stop(0.1, 0);
                left4bar.setPosition(0.42);
                right4bar.setPosition(0.41);
                stop(2.0, .41);
                claw.setPosition(0.25);
                stop(1.0, .41);
                leftLinkage.setPosition(0.36);
                rightLinkage.setPosition(0.36);
                stop(0.05, .41);
                left4bar.setPosition(0.10);
                right4bar.setPosition(0.09);
            }
            if(gamepad1.dpad_left) {//X button
                left4bar.setPosition(0.44);
                right4bar.setPosition(0.43);
                stop(0.1, 0.44);
                leftLinkage.setPosition(0.21);
                rightLinkage.setPosition(0.21);
                /*stop(2.0, .45);
                claw.setPosition(0.25);
                stop(1.0, .45);
                leftLinkage.setPosition(0.36);
                rightLinkage.setPosition(0.36);
                stop(0.05, .45);
                left4bar.setPosition(0.10);
                right4bar.setPosition(0.09);*/
         /*   }
            if(gamepad1.dpad_down) {//X button
                left4bar.setPosition(.46);
                right4bar.setPosition(.45);
                stop(0.1, 0.46);
                leftLinkage.setPosition(0.21);
                rightLinkage.setPosition(0.21);
                /*stop(2.0, .46);
                claw.setPosition(0.25);
                stop(1.0, .46);
                leftLinkage.setPosition(0.36);
                rightLinkage.setPosition(0.36);
                stop(0.05, .46);
                left4bar.setPosition(0.10);
                right4bar.setPosition(0.09);*/
          /*  }
            if(gamepad1.right_trigger > 0.2) {//O but
                leftLinkage.setPosition(0.215);
                rightLinkage.setPosition(0.215);
            }
            //Cone 5 -  .35/.34 (Topmost cone)
            //Cone 4 - .39/.38
            //Cone 3 - .42/.41
            //Cone 2 - .44/.43
            //Cone 1 - .49/.48

        */}

    }

    public void stop(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run) {
        }
    }
    public void stop2(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run) {
            left4bar.setPosition(.10);
            right4bar.setPosition(.09);
        }
    }
    public void stop3(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run) {
            left4bar.setPosition(.25);
            right4bar.setPosition(.25);
        }
    }
    public void stop5(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run) {
            leftLinkage.setPosition(0);
            rightLinkage.setPosition(0);
        }
    }

    public void moveArmUp(double pos, double speed) {
        while(((-1*leftSlide.getCurrentPosition()) < pos) || ((-1*rightSlide.getCurrentPosition()) < pos)) {
            leftSlide.setPower(speed);
            rightSlide.setPower(speed);
        }
        holdArm();
    }

    public void moveArmDown(double pos, double speed) {
        while(((-1*leftSlide.getCurrentPosition()) > pos) || ((-1*rightSlide.getCurrentPosition()) > pos)) {
            leftSlide.setPower(-speed);
            rightSlide.setPower(-speed);
        }
        holdArm();
    }

    public void holdArm() {
        leftSlide.setPower(0.1);
        rightSlide.setPower(0.1);
    }
    public void holdSlides(double time) {
        double run = (runtime.time()+time);
        while(runtime.time() < run){
            leftSlide.setPower(0.1);
            rightSlide.setPower(0.1);
        }
    }
}