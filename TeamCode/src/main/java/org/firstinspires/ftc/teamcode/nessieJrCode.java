package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.util.*;
import java.lang.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DigitalChannel;

//@Disabled
@TeleOp
public class nessieJrCode extends LinearOpMode {

    //declare Drive Motors
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;

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
    double joyScale = 0.8;
    double joyScale2 = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize Drive Motors
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        //Initialize Drive Motors' Directions
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("DRAGON: ", "I am in super duper extremely extreme agony extreme");
        telemetry.update();

        //Wait to start code
        waitForStart();

        // Repeatedly run code in here until stop button is pressed
        //All motors are backwards except ones that are used
        while (opModeIsActive()) {

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
            LF -= X2;
            RF += X2;
            LB += X2;
            RB -= X2;

            //Turning
            LF -= X1;
            RF += X1;
            LB -= X1;
            RB += X1;


            //Set Motors
            motorFrontLeft.setPower(LF);
            motorFrontRight.setPower(RF);
            motorBackLeft.setPower(LB);
            motorBackRight.setPower(RB);
        }
    }
}