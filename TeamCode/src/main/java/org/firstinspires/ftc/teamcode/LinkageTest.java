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
public class LinkageTest extends LinearOpMode {
    private Servo rightLinkage;
    private Servo leftLinkage;
    private Servo claw;

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

        claw = hardwareMap.servo. get("claw");

        leftLinkage.setDirection(Servo.Direction.FORWARD);
        rightLinkage.setDirection(Servo.Direction.REVERSE);


        //Wait to start code
        waitForStart();

        // Repeatedly run code in here until stop button is pressed
        //All motors are backwards except ones that are used
        while (opModeIsActive()) {


            if (gamepad1.y) {
                claw.setPosition(0);
            }
            else if (gamepad1.a) {
                claw.setPosition(0.05);
            }

        }

    }
}