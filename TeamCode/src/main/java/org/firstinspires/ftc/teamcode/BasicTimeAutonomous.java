//Time Based Parking Autonomous

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import java.util.Iterator;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import java.security.Guard;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
//import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous

public class BasicTimeAutonomous extends LinearOpMode {

    //Declare Drive Motors
    private DcMotor motorfrontLeft;
    private DcMotor motorfrontRight;
    private DcMotor motorbackLeft;
    private DcMotor motorbackRight;

    //The Time Object
    private ElapsedTime runtime=new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        //Initialize drive motors
        motorfrontLeft = hardwareMap.dcMotor.get("motorfrontLeft");
        motorfrontRight = hardwareMap.dcMotor.get("motorfrontRight");
        motorbackLeft = hardwareMap.dcMotor.get("motorbackLeft");
        motorbackRight = hardwareMap.dcMotor.get("motorbackRight");

        //Initialize drive motors' direction
        motorfrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorfrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorbackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorbackRight.setDirection(DcMotor.Direction.FORWARD);

        //moveForward(0.5);
        //Stop(1.0);
    }

    /** FUNCTIONS */

    /** TIME-BASED */

    //Move backwards
    public void moveBack(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(-1);
            motorfrontRight.setPower(-1);
            motorbackLeft.setPower(-1);
            motorbackRight.setPower(-1);
        }
    }

    //Move forwards
    public void moveForward(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(1);
            motorfrontRight.setPower(1);
            motorbackLeft.setPower(1);
            motorbackRight.setPower(1);
        }
    }

    //Turn right
    public void turnRight(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(1);
            motorfrontRight.setPower(-1);
            motorbackLeft.setPower(1);
            motorbackRight.setPower(-1);
        }
    }

    //Turn left
    public void turnLeft(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(-1);
            motorfrontRight.setPower(1);
            motorbackLeft.setPower(-1);
            motorbackRight.setPower(1);
        }
    }

    //Strafe right
    public void strafeRight(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(-1);
            motorfrontRight.setPower(1);
            motorbackLeft.setPower(1);
            motorbackRight.setPower(-1);
        }
    }

    //Strafe left
    public void strafeLeft(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(1);
            motorfrontRight.setPower(-1);
            motorbackLeft.setPower(-1);
            motorbackRight.setPower(1);
        }
    }

    public void Stop(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(0);
            motorfrontRight.setPower(0);
            motorbackLeft.setPower(0);
            motorbackRight.setPower(0);
            //CarouselMotor.setPower(0);
        }
    }
}