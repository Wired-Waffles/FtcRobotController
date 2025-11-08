package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagVision;
import org.firstinspires.ftc.teamcode.mechanisms.DriveChainTank;
import org.firstinspires.ftc.teamcode.mechanisms.RubberBandIntake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "TeleOP main")
public class TeleOP extends LinearOpMode {
    //redoing for decode <3
    //using new mechanisms package cuz cleaner

    //mech obj declare
    DriveChainTank driveChain = new DriveChainTank();
    AprilTagVision aprilTagVision = new AprilTagVision();
    RubberBandIntake intake = new RubberBandIntake();
    //everything bloody else
    final double DESIRED_DISTANCE = 24.0; //this is how close the camera should get to the target (inches)

    //vision constants from external.samples.RobotAutoDriveToAprilTagTank example op mode
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN  =   0.01 ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;    // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    double drivePower;
    double turn;
    @Override
    public void runOpMode() throws InterruptedException {
        boolean targetFound     = false;
        driveChain.init(hardwareMap);
        aprilTagVision.init(hardwareMap);
        intake.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {

            //auto aim via april tag
            if (gamepad1.left_bumper && targetFound) {
                //find heading and range error to help calc the robots movement
                double  rangeError   = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError = desiredTag.ftcPose.bearing;

                //use the speed and turn gain constants times the errors to calc robot movement.
                // clip to max
                drivePower = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn  = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;

                telemetry.addData("Auto","Drive %5.2f, Turn %5.2f", drivePower, turn);
            } else {
                //default single joystick tank drive
                drivePower = -gamepad1.left_stick_y  / 1.5;  // Reduce drive rate to 50%.
                turn  = gamepad1.right_stick_x / 2.0;  // Reduce turn rate to 25%.
                telemetry.addData("Manual","Drive %5.2f, Turn %5.2f", drivePower, turn);
            }

            driveChain.moveRobot(drivePower, turn);
            telemetry.update();

            //intake motor
            if (gamepad1.right_bumper) {
                intake.intakePower(1);
            } else if (gamepad2.cross) {
                intake.eject();
            } else {
                intake.intakePower(0);
            }
        }
    }
}