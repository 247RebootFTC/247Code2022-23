package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.*;
import java.lang.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp (name="Reuben's Wild Ride")
public class ReubensWildRide extends LinearOpMode {

    //This is where we set all of our variables so we can call them in future code
    double tgtPower = 0;

    //declare Drive Motors
    private DcMotor motorfrontLeft;
    private DcMotor motorfrontRight;
    private DcMotor motorbackLeft;
    private DcMotor motorbackRight;

    //Declare Mechanism Motors
    //private DcMotor ArmMotor;
    //private DcMotor CarouselMotor;
    //private DcMotor IntakeMotor;
    //private DcMotor TotemArm;

    //Declare Servos
    //private Servo GrabberServo;
    //private CRServo leftEye;
    //private CRServo rightEye;
    //private CRServo leftTusk;
    //private CRServo rightTusk;
    //time variable
    private ElapsedTime runtime = new ElapsedTime();

    //motor speed variables
    double LF;
    double RF;
    double LB;
    double RB;
    //double TL;
    //double TR;
    //POWER
    double pwr = 3/4;

    //Joystick position variables
    double X1;
    double Y1;
    double X2;
    double Y2;
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
        // Initialize drive motors
        motorfrontLeft = hardwareMap.dcMotor.get("motorfrontLeft");
        motorfrontRight = hardwareMap.dcMotor.get("motorfrontRight");
        motorbackLeft = hardwareMap.dcMotor.get("motorbackLeft");
        motorbackRight = hardwareMap.dcMotor.get("motorbackRight");

        //Initialize other motors
        //ArmMotor = hardwareMap.dcMotor.get("ArmMotor");
        //CarouselMotor = hardwareMap.dcMotor.get("CarouselMotor");
        //IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
        //TotemArm = hardwareMap.dcMotor.get("TotemArm");

        //Initialize servos
        //GrabberServo = hardwareMap.servo.get("GrabberServo");
        //leftEye = hardwareMap.crservo.get("leftEye");
        //rightEye = hardwareMap.crservo.get("rightEye");
        //leftTusk = hardwareMap.crservo.get("leftTusk");
        //rightTusk = hardwareMap.crservo.get("rightTusk");

        //Initialize drive motors' direction
        //DONT CHANGE THIS CONFIRGURATION
        motorfrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorfrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorbackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorbackRight.setDirection(DcMotor.Direction.REVERSE);

        //Initialize other motors' directions
        //ArmMotor.setDirection(DcMotor.Direction.FORWARD);
        //CarouselMotor.setDirection(DcMotor.Direction.REVERSE);
        //IntakeMotor.setDirection(DcMotor.Direction.FORWARD);
        //TotemArm.setDirection(DcMotor.Direction.FORWARD);

        //Wait to start code
        waitForStart();

        // Repeatedly run code in here until stop button is pressed
        //All motors are backwards except ones that are used
        while (opModeIsActive()) {
            //all the drive code

            //reset speed variables
            LF = 0;
            RF = 0;
            LB = 0;
            RB = 0;
            /*D1 = 0;
            D2 = 0;*/

            //get joystick values
             Y1 = Math.pow((double) gamepad1.right_stick_y * joyScale, pwr);
             X1 = Math.pow((double) gamepad1.right_stick_x * joyScale, pwr);
             Y2 = Math.pow((double) gamepad1.left_stick_y * joyScale, pwr);
             X2 = Math.pow((double) gamepad1.left_stick_x * joyScale, pwr);


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
            LF += Y2;
            RF += Y2;
            LB += Y2;
            RB += Y2;

            //Strafing
            LF -=  X2;
            RF +=  X2;
            LB +=  X2;
            RB -=  X2;

            //Turning
            //turn right analog is imperfect
            LF -= 0.75*X1;
            RF += 0.75*X1;
            LB -= 0.75*X1;
            RB += 0.75*X1;
            //Wheel power limiter
            /*LF = Math.max(-motorMax, Math.min(LF, motorMax));
            RF = Math.max(-motorMax, Math.min(RF, motorMax));
            LB = Math.max(-motorMax, Math.min(LB, motorMax));
            RB = Math.max(-motorMax, Math.min(RB, motorMax));*/


            //set motors
            motorfrontLeft.setPower(LF);
            motorfrontRight.setPower(RF);
            motorbackLeft.setPower(LB);
            motorbackRight.setPower(RB);

            //Carousel Motor Code
            /*if(gamepad2.x){
                CarouselMotor.setPower(1);
            }
            else if(gamepad2.b){
                CarouselMotor.setPower(-1);
            }
            else{
                CarouselMotor.setPower(0);
            }

            if(gamepad2.left_stick_y > 0)
            {
                ArmMotor.setPower(0.5);
            }
            else if(gamepad2.left_stick_y < 0){
                ArmMotor.setPower(-1);
            }
            else{
                ArmMotor.setPower(0);
            }

            //Intake
            if(gamepad2.right_stick_y > 0){
                IntakeMotor.setPower(-0.75);
            }
            //Outtake
            else if (gamepad2.right_stick_y < 0){
                IntakeMotor.setPower(0.75);
            }
            else{
                IntakeMotor.setPower(0);
            }

            //Left Eye Forward
            if (gamepad2.left_trigger > 0.2) {
                leftEye.setPower(1);
            }
            //Left Eye Backward
            else if (gamepad2.left_bumper) {
                leftEye.setPower(-1);
            }
            else {
                leftEye.setPower(0);
            }

            //Right Eye Forward
            if (gamepad2.right_trigger > 0.2) {
                rightEye.setPower(-1);
            }
            //Right Eye Backward
            else if (gamepad2.right_bumper) {
                rightEye.setPower(1);
            }
            else {
                rightEye.setPower(0);
            }

            if (gamepad2.dpad_up){
                leftTusk.setPower(1);
            }
            else if (gamepad2.dpad_down){
                leftTusk.setPower(-1);
            }
            else {
                leftTusk.setPower(0);
            }

            if (gamepad2.dpad_right){
                rightTusk.setPower(1);
            }
            else if (gamepad2.dpad_left){
                rightTusk.setPower(-1);
            }
            else {
                rightTusk.setPower(0);
            }*/


            //Totem Arm Code
            /*if (gamepad2.dpad_down) {
                TotemArm.setPower(-1);
            }
            else if (gamepad2.dpad_up) {
                TotemArm.setPower(1);
            }
            else {
                TotemArm.setPower(0);
            }

            //Grabber Servo Code
            else if (gamepad2.dpad_right) {
                GrabberServo.setPosition(0.8);
            }
            else {
                GrabberServo.setPosition(0.4);
            }*/

            /*while (opModeIsActive()) {
                if (limiter.getState() == true) {
                    WobbleGoalMotor.setPower(0);
                }
            }*/
        }
    }
}
