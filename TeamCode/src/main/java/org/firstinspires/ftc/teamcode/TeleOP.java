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
//hi adi
    private DcMotor left_motor;
    private DcMotor right_motor;
    private DcMotor armlift;
    private DcMotor viper;
    private Servo grip;
    private CRServo yaw;
    private CRServo Viperspin;

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

    @Override
    public void runOpMode() {
        left_motor = hardwareMap.get(DcMotor.class, "left_motor");
        right_motor = hardwareMap.get(DcMotor.class, "right_motor");
        armlift = hardwareMap.get(DcMotor.class, "arm lift");
        viper = hardwareMap.get(DcMotor.class, "viper");
        grip = hardwareMap.get(Servo.class, "grip");
        yaw = hardwareMap.get(CRServo.class, "yaw");
        Viperspin = hardwareMap.get(CRServo.class, "Viperspin");


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
                if (gamepad1.options) {
                    speed_mod -= 0.3;
                }
                if (gamepad1.left_bumper && gamepad1.touchpad_finger_1) {
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
                //claw
                telemetry.addData("small arm lift", viper2);
                if (gamepad2.square) {
                    grip.setPosition(0.3);
                } else {
                    grip.setPosition(0);
                }
                //yaw
                if (gamepad2.dpad_left) { yaw.setPower(1); } else {yaw.setPower(0);}
                if (gamepad2.dpad_right) { yaw.setPower(-1); } else {yaw.setPower(0);}
                // Jaw (viperspin)
                if (gamepad2.dpad_up) { Viperspin.setPower(1); } else {Viperspin.setPower(0);}
                if (gamepad2.dpad_down) {Viperspin.setPower(-1); } else {Viperspin.setPower(0);}
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