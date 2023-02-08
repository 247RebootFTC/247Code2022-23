package org.firstinspires.ftc.teamcode;
//imports libraries
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp (name="BasicMovement")
public class BasicMovement extends LinearOpMode {
    //Drive motors
    private DcMotor motorfrontLeft;
    private DcMotor motorfrontRight;
    private DcMotor motorbackLeft;
    private DcMotor motorbackRight;

    //Other motors
    //private DcMotor IntakeMotor;
    //private DcMotor CarouselMotor;
    //private DcMotor ArmMotor;

    //CODE STARTS HERE
    //@Override
    public void runOpMode() throws InterruptedException {
        //Initializes drive motors
        motorfrontLeft = hardwareMap.dcMotor.get("motorfrontLeft");
        motorfrontRight = hardwareMap.dcMotor.get("motorfrontRight");
        motorbackLeft = hardwareMap.dcMotor.get("motorbackLeft");
        motorbackRight = hardwareMap.dcMotor.get("motorbackRight");
        //Initializes motor directions
        motorfrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorfrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorbackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorbackRight.setDirection(DcMotor.Direction.REVERSE);
        //Initializes Other Motors
        //CarouselMotor = hardwareMap.dcMotor.get("CarouselMotor");
        //IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
        //ArmMotor = hardwareMap.dcMotor.get("ArmMotor");
        //Initializes motor directions
        //ArmMotor.setDirection(DcMotor.Direction.REVERSE);
        //CarouselMotor.setDirection(DcMotor.Direction.REVERSE);
        //IntakeMotor.setDirection(DcMotor.Direction.FORWARD);

        //run portion
        waitForStart();
        while (opModeIsActive()) {
            //DRIVER
            //Move forward
            if (gamepad1.left_stick_y < 0) {
                motorfrontLeft.setPower(1);
                motorfrontRight.setPower(1);
                motorbackLeft.setPower(1);
                motorbackRight.setPower(1);
                //telemetry for testing if the code is working (troubleshooting)
                telemetry.addLine("Forward");
                telemetry.update();
            }
            //Move back
            else if (gamepad1.left_stick_y > 0) {
                motorfrontLeft.setPower(-1);
                motorfrontRight.setPower(-1);
                motorbackLeft.setPower(-1);
                motorbackRight.setPower(-1);
                //telemetry
                telemetry.addLine("Backwards");
                telemetry.update();
            }
            //if y direction not moved power=0
            else {
                motorfrontLeft.setPower(0);
                motorfrontRight.setPower(0);
                motorbackLeft.setPower(0);
                motorbackRight.setPower(0);
            }
            //Strafe right
            if (gamepad1.dpad_right) {
                motorfrontLeft.setPower(-1);
                motorfrontRight.setPower(1);
                motorbackLeft.setPower(1);
                motorbackRight.setPower(-1);
                //telemetry
                telemetry.addLine("RightStrafe");
                telemetry.update();
            }
            //Strafe left
            else if (gamepad1.dpad_left) {
                motorfrontLeft.setPower(1);
                motorfrontRight.setPower(-1);
                motorbackLeft.setPower(-1);
                motorbackRight.setPower(1);
                //telemetry
                telemetry.addLine("LeftStrafe");
                telemetry.update();
            }
            //if dpad not touched power=0
            else {
                motorfrontLeft.setPower(0);
                motorfrontRight.setPower(0);
                motorbackLeft.setPower(0);
                motorbackRight.setPower(0);
            }
            if (gamepad1.right_stick_x > 0) {
                motorfrontLeft.setPower(1);
                motorfrontRight.setPower(-1);
                motorbackLeft.setPower(1);
                motorbackRight.setPower(-1);
                telemetry.addLine("??");
                telemetry.update();
            } else if (gamepad1.right_stick_x < 0) {
                motorfrontLeft.setPower(-1);
                motorfrontRight.setPower(1);
                motorbackLeft.setPower(-1);
                motorbackRight.setPower(1);
                telemetry.addLine("??");
                telemetry.update();
            } else {
                motorfrontLeft.setPower(0);
                motorfrontRight.setPower(0);
                motorbackLeft.setPower(0);
                motorbackRight.setPower(0);
            }
            //turn right
            /*if (gamepad1.right_stick_x > 0) {
                motorfrontLeft.setPower(1);
                motorbackLeft.setPower(1);
                motorfrontRight.setPower(-1);
                motorbackRight.setPower(-1);
                telemetry.addLine("Right");
                telemetry.update();
            }
            //turn left
            else if (gamepad1.right_stick_x < 0) {
                motorfrontRight.setPower(1);
                motorbackRight.setPower(1);
                motorfrontLeft.setPower(-1);
                motorbackLeft.setPower(-1);
                telemetry.addLine("Left");
                telemetry.update();
            }
            //if stick not touched in x direction power=0
            else {
                motorfrontLeft.setPower(0);
                motorfrontRight.setPower(0);
                motorbackLeft.setPower(0);
                motorbackRight.setPower(0);
            }*/
            //ALL CODE HERE MOVES @25% speed
            //Move forward
            /*if (gamepad1.left_stick_y > 0 && gamepad1.a) {
                motorfrontLeft.setPower(1);
                motorfrontRight.setPower(1);
                motorbackLeft.setPower(1);
                motorbackRight.setPower(1);
            }
            //Move back
            else if (gamepad1.left_stick_y < 0 && gamepad1.a) {
                motorfrontLeft.setPower(-1);
                motorfrontRight.setPower(-1);
                motorbackLeft.setPower(-1);
                motorbackRight.setPower(-1);
            }
            //if y direction not moved power=0
            else {
                motorfrontLeft.setPower(0);
                motorfrontRight.setPower(0);
                motorbackLeft.setPower(0);
                motorbackRight.setPower(0);
            }
            //Strafe right
            if (gamepad1.dpad_right && gamepad1.a) {
                motorfrontLeft.setPower(1);
                motorfrontRight.setPower(-1);
                motorbackLeft.setPower(-1);
                motorbackRight.setPower(1);
            }
            //Strafe left
            else if (gamepad1.dpad_left && gamepad1.a) {
                motorfrontLeft.setPower(-1);
                motorfrontRight.setPower(1);
                motorbackLeft.setPower(1);
                motorbackRight.setPower(-1);
            }
            //if dpad not touched power=0
            else {
                motorfrontLeft.setPower(0);
                motorfrontRight.setPower(0);
                motorbackLeft.setPower(0);
                motorbackRight.setPower(0);
            }
            //turn right
            if (gamepad1.right_stick_x > 0 && gamepad1.a) {
                motorfrontLeft.setPower(-1);
                motorbackLeft.setPower(-1);
                motorfrontRight.setPower(1);
                motorbackRight.setPower(1);
            }
            //turn left
            else if (gamepad1.right_stick_x < 0 && gamepad1.a) {
                motorfrontRight.setPower(-1);
                motorbackRight.setPower(-1);
                motorfrontLeft.setPower(1);
                motorbackLeft.setPower(1);
            }
            //if stick not touched in x direction power=0
            else {
                motorfrontLeft.setPower(0);
                motorfrontRight.setPower(0);
                motorbackLeft.setPower(0);
                motorbackRight.setPower(0);
            }
            */
            //GUNNER

            //CarouselMotor a
            //ArmMotor dpad up/down
            //Intake Motor Gamepad stick up/y>0 down/y<0
            /*
            if (gamepad2.a) {
                CarouselMotor.setPower(1);
            } else if (gamepad2.b) {
                CarouselMotor.setPower(-1);
            } else {
                CarouselMotor.setPower(0);
            }

            if (gamepad2.dpad_up) {
                ArmMotor.setPower(0.5);
            } else if (gamepad2.dpad_down) {
                ArmMotor.setPower(-0.5);
            } else {
                ArmMotor.setPower(0);
            }

            //Intake
            if (gamepad2.right_stick_y > 0) {
                IntakeMotor.setPower(1);

            }
            //Outtake
            else if (gamepad2.right_stick_y < 0) {
                IntakeMotor.setPower(-1);
            } else {
                IntakeMotor.setPower(0);
            }
            */

        }
    }
}