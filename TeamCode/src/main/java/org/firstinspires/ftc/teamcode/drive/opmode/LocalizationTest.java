package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*X MULTIPLIER*/
    //Measured Distance: 100 inches
    //Attempt 1: 99.58306210601498
    //Attempt 2: 99.74408134370914
    //Attempt 3: 99.5107017029
    //Attempt 4: 99.9349040544
    //Multiplier 1: 1.00418683544
    //Multiplier 2: 1.0025657528
    //Multiplier 3: 1.00491704197
    //Multiplier 4: 1.00065138348
    //Final Multiplier: 1.00308025342

/*Y MULTIPLIER*/
    //Measured Distance: 100 inches
    //Attempt 1: 98.68496
    //Attempt 2: 99.53568
    //Attempt 3: 98.53205
    //Attempt 4: 99.21530
    //Multiplier 1: 1.01332563746
    //Multiplier 2: 1.00466485988
    //Multiplier 3: 1.0148981981
    //Multiplier 4: 1.00790906241
    //Final Multiplier: 1.01019943946
/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Disabled
@TeleOp (group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
