//NO

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.Servo.MAX_POSITION;
import static com.qualcomm.robotcore.hardware.Servo.MIN_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@TeleOp
public class BasicServoCode extends LinearOpMode {

    //Declare Regular Servos
    private Servo servo;
    private CRServo crServo;

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize Regular Servos
        servo = hardwareMap.servo.get("servo");
        crServo = hardwareMap.crservo.get("crServo");

        //Initialize Servos' Directions
        servo.setDirection(Servo.Direction.FORWARD);
        crServo.setDirection(DcMotorSimple.Direction.FORWARD);

        //Wait to start code
        waitForStart();

        // Repeatedly run code in here until stop button is pressed
        //All motors are backwards except ones that are used
        while (opModeIsActive()) {
            if(gamepad1.a) {
                servo.setPosition(0.0);
                crServo.setPower(0.0);
            }
            if(gamepad1.b) {
                servo.setPosition(1.0);
                crServo.setPower(1.0);
            }
        }

    }
}