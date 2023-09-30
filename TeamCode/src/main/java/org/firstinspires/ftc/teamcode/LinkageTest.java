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


@TeleOp
public class LinkageTest extends LinearOpMode {
    private Servo rightLinkage;
    private Servo leftLinkage;
    private Servo leftForebar;
    private Servo rightForebar;
    private Servo left4bar;
    private Servo right4bar;
    private Servo claw;
    private CRServo intake;

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
    double joyScale = 0.6;
    double joyScale2 = 0.6;
    double motorMax = 0.9;

    @Override
    public void runOpMode() throws InterruptedException {
        leftLinkage =  hardwareMap.servo.get("leftLinkage");
        rightLinkage = hardwareMap.servo. get("rightLinkage");

        leftForebar =  hardwareMap.servo.get("leftForebar");
        rightForebar = hardwareMap.servo. get("rightForebar");

        left4bar =  hardwareMap.servo.get("left4bar");
        right4bar = hardwareMap.servo. get("right4bar");

        claw = hardwareMap.servo. get("claw");

        leftLinkage.setDirection(Servo.Direction.REVERSE);
        rightLinkage.setDirection(Servo.Direction.FORWARD);

        leftForebar.setDirection(Direction.REVERSE);
        rightForebar.setDirection(Direction.FORWARD);

        left4bar.setDirection(Servo.Direction.REVERSE);
        right4bar.setDirection(Servo.Direction.FORWARD);

        intake = hardwareMap.crservo.get("intake");

        //Wait to start code
        waitForStart();

        // Repeatedly run code in here until stop button is pressed
        //All motors are backwards except ones that are used
        while (opModeIsActive()) {
            if(gamepad1.a) {
                leftForebar.setPosition(0.3);
                rightForebar.setPosition(0.3);
            }
            else if(gamepad1.y) {
                leftForebar.setPosition(0.8);
                rightForebar.setPosition(0.8);
            }
            else if(gamepad1.b) {
                leftForebar.setPosition(0.5);
                rightForebar.setPosition(0.5);
            }
            else if(gamepad1.x) {
                leftForebar.setPosition(0);
                rightForebar.setPosition(0);
            }

            if(gamepad1.right_bumper) {
                intake.setPower(-1.0);
            }

            if(gamepad1.left_bumper) {
                intake.setPower(1.0);
            }


        }

    }
}