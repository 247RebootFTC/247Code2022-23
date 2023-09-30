//Digital Chassis Movement

package org.firstinspires.ftc.teamcode;
//imports libraries
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp
public class BasicMovement extends LinearOpMode {

    //Declare Drive Motors
    private DcMotor motorfrontLeft;
    private DcMotor motorfrontRight;
    private DcMotor motorbackLeft;
    private DcMotor motorbackRight;

    public void runOpMode() throws InterruptedException {

        //Initializes Drive Motors
        motorfrontLeft = hardwareMap.dcMotor.get("motorfrontLeft");
        motorfrontRight = hardwareMap.dcMotor.get("motorfrontRight");
        motorbackLeft = hardwareMap.dcMotor.get("motorbackLeft");
        motorbackRight = hardwareMap.dcMotor.get("motorbackRight");

        //Set Motors' Directions - left should be the same and right should be the same
        motorfrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorfrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorbackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorbackRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            //Move forward
            if (gamepad1.left_stick_y < 0) {
                motorfrontLeft.setPower(1.0);
                motorfrontRight.setPower(1.0);
                motorbackLeft.setPower(1.0);
                motorbackRight.setPower(1.0);
            }

            //Move backward
            else if (gamepad1.left_stick_y > 0) {
                motorfrontLeft.setPower(-1.0);
                motorfrontRight.setPower(-1.0);
                motorbackLeft.setPower(-1.0);
                motorbackRight.setPower(-1.0);
            }

            //Strafe right - diagonals should be the same
            else if (gamepad1.dpad_right) {
                motorfrontLeft.setPower(-1.0);
                motorfrontRight.setPower(1.0);
                motorbackLeft.setPower(1.0);
                motorbackRight.setPower(-1.0);
            }

            //Strafe left - diagonals should be the same
            else if (gamepad1.dpad_left) {
                motorfrontLeft.setPower(1.0);
                motorfrontRight.setPower(-1.0);
                motorbackLeft.setPower(-1.0);
                motorbackRight.setPower(1.0);
            }

            //Turn right
            else if (gamepad1.right_stick_x > 0) {
                motorfrontLeft.setPower(1);
                motorfrontRight.setPower(-1);
                motorbackLeft.setPower(1);
                motorbackRight.setPower(-1);
            }

            //Turn left
            else if (gamepad1.right_stick_x < 0) {
                motorfrontLeft.setPower(-1);
                motorfrontRight.setPower(1);
                motorbackLeft.setPower(-1);
                motorbackRight.setPower(1);
            }

            //Robot doesn't move if no buttons are pressed
            else {
                motorfrontLeft.setPower(0);
                motorfrontRight.setPower(0);
                motorbackLeft.setPower(0);
                motorbackRight.setPower(0);
            }

        }
    }
}