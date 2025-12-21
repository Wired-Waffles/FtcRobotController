package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagRead;
import org.firstinspires.ftc.teamcode.mechanisms.DriveChainTank;
import org.firstinspires.ftc.teamcode.mechanisms.FlywheelShooter;
import org.firstinspires.ftc.teamcode.mechanisms.RubberBandIntake;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "TeleOP main")
public class TeleOP extends LinearOpMode {
    //redoing for decode <3
    //using new mechanisms package cuz cleaner

    //mech obj declare
    DriveChainTank driveChain = new DriveChainTank();
    RubberBandIntake intake = new RubberBandIntake();
    AprilTagRead aprilTagReader = new AprilTagRead();

    FlywheelShooter shooter = new FlywheelShooter();
    //everything bloody else
    double drivePower;
    double turn;

    double driveMod = 1.5;
    enum controlMode {
        RED_AUTO_AIM,
        BLUE_AUTO_AIM,
        BOTH_SOMEHOW,
        MANUAL;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        boolean targetFound     = false;
        driveChain.init(hardwareMap);
        intake.init(hardwareMap);
        aprilTagReader.init(hardwareMap, telemetry);
        shooter.init(hardwareMap);
        waitForStart();
        controlMode controlCase = controlMode.MANUAL;
        while (opModeIsActive()) {
            aprilTagReader.update();
            AprilTagDetection id20 = aprilTagReader.getTagByID(20);
            AprilTagDetection id24 = aprilTagReader.getTagByID(24);
            aprilTagReader.displayTelemetry(id20);
            aprilTagReader.displayTelemetry(id24);
            if (gamepad1.right_bumper) {
                driveMod = 1;
            }
            drivePower = -gamepad1.left_stick_y  / driveMod;
            //turn  = gamepad1.right_stick_x / 2.0;

            if (id20 != null && id24 != null){
                controlCase = controlMode.MANUAL;
            }
            else if (gamepad1.right_bumper && id24 != null) {
                controlCase = controlMode.BLUE_AUTO_AIM;
            } else if (gamepad1.right_bumper && id20 != null) {
                controlCase = controlMode.RED_AUTO_AIM;
            } else {
                controlCase = controlMode.MANUAL;
            }
            switch (controlCase) {
                case RED_AUTO_AIM:
                    if (id20.ftcPose.x > 5) {
                        turn = 0.2;
                    } else if (id20.ftcPose.x < -5) {
                        turn = -0.2;
                    }
                case BLUE_AUTO_AIM:
                    if (id24.ftcPose.x > 5) {
                        turn = 0.2;
                    } else if (id24.ftcPose.x < -5) {
                        turn = -0.2;
                    }
                default:
                    turn  = gamepad1.right_stick_x / 2.0;
            }

            //intake motor
            if (gamepad2.dpad_up) {
                intake.intakePower(1);
            } else if (gamepad2.a) {
                intake.eject();
            } else {
                intake.intakePower(0);
            }

            if (gamepad2.circle) {
                shooter.start(1700);
                intake.intakePower(0);
            } else if(gamepad2.triangle) {
                shooter.start(2130);
            } else if (gamepad2.cross) {
                shooter.kill();
            }
            driveChain.moveRobot(drivePower, turn);
            telemetry.addData("Exposure support", aprilTagReader.getExpoSprt());
            telemetry.addData("Max gain", aprilTagReader.getMaxGain());
            telemetry.addData("Min Gain", aprilTagReader.getMinGain());
            telemetry.addData("Drive","Drive %5.2f, Turn %5.2f", drivePower, turn);
            telemetry.addData("Flywheel speed", shooter.getVelo());
            telemetry.update();
        }
    }
}