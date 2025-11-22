package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagRead;
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
    RubberBandIntake intake = new RubberBandIntake();
    AprilTagRead aprilTagReader = new AprilTagRead();
    //everything bloody else
    double drivePower;
    double turn;

    double driveMod = 1.5;
    @Override
    public void runOpMode() throws InterruptedException {
        boolean targetFound     = false;
        driveChain.init(hardwareMap);
        intake.init(hardwareMap);
        aprilTagReader.init(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()) {
            aprilTagReader.update();
            AprilTagDetection id20 = aprilTagReader.getTagByID(20);
            aprilTagReader.displayTelemetry(id20);
            if (gamepad1.options) {
                driveMod = 1;
            }
            drivePower = -gamepad1.left_stick_y  / driveMod;
            turn  = gamepad1.right_stick_x / 2.0;

            telemetry.addData("Manual","Drive %5.2f, Turn %5.2f", drivePower, turn);
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