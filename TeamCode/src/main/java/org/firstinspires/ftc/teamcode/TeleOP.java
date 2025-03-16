package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Dual Controller")
public class TeleOP extends LinearOpMode {

    private DcMotor left_motor;
    private DcMotor right_motor;
    private DcMotor armlift;
    private DcMotor viper;
    private CRServo spinner;
    private CRServo spinner2;
    private Servo basket;
    private Servo grip;

    boolean USE_WEBCAM;
    AprilTagProcessor myAprilTagProcessor;
    Position cameraPosition;
    YawPitchRollAngles cameraOrientation;

    boolean gripperOpen;
    double speed_mod;
    double turn;
    double drive_motor;
    double arm_lift_speedmod;
    double arm_lift;
    int smallarmlift_speedmod;
    double viper2;

    /**
     * Display info (using telemetry) for a recognized AprilTag.
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> myAprilTagDetections;
        AprilTagDetection myAprilTagDetection;

        // Get a list of AprilTag detections.
        myAprilTagDetections = myAprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
        // Iterate through list and call a function to display info for each recognized AprilTag.
        for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection_item;
            // Display info about the detection.
            telemetry.addLine("");
            if (myAprilTagDetection.metadata != null) {
                telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
                telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().z, 6, 1) + "  (inch)");
                telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getOrientation().getPitch(), 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getOrientation().getRoll(), 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getOrientation().getYaw(), 6, 1) + "  (deg)");
            } else {
                telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
                telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + "" + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
            }
        }
        telemetry.addLine("");
        telemetry.addLine("key:");
        telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    }

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        left_motor = hardwareMap.get(DcMotor.class, "left_motor");
        right_motor = hardwareMap.get(DcMotor.class, "right_motor");
        armlift = hardwareMap.get(DcMotor.class, "arm lift");
        viper = hardwareMap.get(DcMotor.class, "viper");
        spinner = hardwareMap.get(CRServo.class, "spinner");
        spinner2 = hardwareMap.get(CRServo.class, "spinner2");
        basket = hardwareMap.get(Servo.class, "basket");
        grip = hardwareMap.get(Servo.class, "grip");

        USE_WEBCAM = true;
        // Put initialization blocks here.
        cameraPosition = new Position(DistanceUnit.CM, 0, 0, 0, 0);
        cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
        // Initialize AprilTag before waitForStart.
        initAprilTag();
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            gripperOpen = true;
            while (opModeIsActive()) {
                // Put loop blocks here.
                speed_mod = 0.5;
                if (gamepad1.options == true) {
                    speed_mod += -0.3;
                }
                if ((gamepad1.left_bumper && gamepad1.touchpad_finger_1) == true) {
                    turn = gamepad1.touchpad_finger_1_x * speed_mod * 3;
                    left_motor.setPower(Math.min(Math.max((drive_motor + turn) * 1, -1), 1));
                    right_motor.setPower(-Math.min(Math.max(drive_motor - turn, -1), 1));
                }
                drive_motor = -gamepad1.left_stick_y * speed_mod;
                turn = gamepad1.right_stick_x * speed_mod;
                left_motor.setPower(Math.min(Math.max((drive_motor + turn) * 1, -1), 1));
                right_motor.setPower(-Math.min(Math.max(drive_motor - turn, -1), 1));
                // arm lift code
                arm_lift_speedmod = 0.5;
                arm_lift = gamepad2.left_stick_y * arm_lift_speedmod * -1;
                armlift.setPower(Math.min(Math.max(arm_lift, -1), 1));
                // small arm lift code
                smallarmlift_speedmod = 1;
                viper2 = (gamepad1.left_trigger - gamepad1.right_trigger) * smallarmlift_speedmod + 0.05;
                viper.setPower(Math.min(Math.max(viper2, -1), 1));
                telemetry.addData("small armlift", viper2);
                // spinner code
                spinner.setPower(gamepad2.right_stick_y);
                spinner2.setPower(-gamepad2.right_stick_y);
                // basket and gripper code
                if (gamepad2.circle) {
                    basket.setPosition(0.3);
                } else {
                    basket.setPosition(-0.2);
                }
                if (gamepad1.cross || gamepad2.square) {
                    grip.setPosition(0.3);
                } else {
                    grip.setPosition(0);
                }
                telemetryAprilTag();
                telemetry.addData("turn value", turn);
                telemetry.addData("target power", drive_motor);
                telemetry.addData("motor power", left_motor.getPower());
                telemetry.update();
            }
        }
    }

    /**
     * Initialize AprilTag Detection.
     */
    private void initAprilTag() {
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;
        VisionPortal myVisionPortal;

        // First, create an AprilTagProcessor.Builder.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        myAprilTagProcessorBuilder.setCameraPose(cameraPosition, cameraOrientation);
        // Create an AprilTagProcessor by calling build.
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();
        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            // Use a webcam.
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            // Use the device's back camera.
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Add myAprilTagProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        // Create a VisionPortal by calling build.
        myVisionPortal = myVisionPortalBuilder.build();
    }
}