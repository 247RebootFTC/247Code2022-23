package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.lang.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DigitalChannel;

@Disabled
@TeleOp
public class BasicMotorEncoderReader extends LinearOpMode {

    //Declare Mechanism Motors
    private DcMotor motor1;
    private DcMotor motor2;

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize Mechanism Motors
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");

        //Set Mechanism Motors' Directions
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);

        initEncoder();

        telemetry.addData("DRAGON: ", "AAAAAAAAAAAAAAAAAAAAAAH");
        telemetry.update();

        //Wait to start code
        waitForStart();


        while (opModeIsActive()) {

            //Outputs encoder values of motors
            telemetry.addLine(String.valueOf(motor1.getCurrentPosition()));
            telemetry.addLine(String.valueOf(motor2.getCurrentPosition()));

            telemetry.update();

            //Motors move in tandem if left joystick is moved up or down
            if (gamepad2.left_stick_y < 0.0) {
                motor1.setPower(-0.5 * gamepad2.left_stick_y);
                motor2.setPower(-0.5 * gamepad2.left_stick_y);
            } else if (gamepad2.left_stick_y > 0.0) {
                motor1.setPower(-0.5 * gamepad2.left_stick_y);
                motor2.setPower(-0.5 * gamepad2.left_stick_y);
            }
        }
    }

    public void initEncoder() {
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}