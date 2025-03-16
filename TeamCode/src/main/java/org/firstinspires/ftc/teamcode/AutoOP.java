package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class AutoOP extends LinearOpMode {
    private DcMotor left_motor;
    private DcMotor right_motor;
    private DcMotor armlift;
    private DcMotor viper;
    private CRServo spinner;
    private CRServo spinner2;
    private Servo basket;
    private Servo grip;


    //this is the actual code, the main method
    @Override
    public void runOpMode() throws InterruptedException {
        left_motor = hardwareMap.get(DcMotor.class, "left_motor");
        right_motor = hardwareMap.get(DcMotor.class, "right_motor");
        AutoOP Main = new AutoOP();

        right_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Setup", "complete");
        telemetry.addData("Ready to start", "Please press the start/play arrow");
        telemetry.update();
        telemetry.clearAll();
        telemetry.addData("right wheel", right_motor.getPower());
        telemetry.addData("left wheel", left_motor.getPower());

        waitForStart();

        while (opModeIsActive()) {
            //execute methods on the object Main <3
            /*right_motor.setPower(1.0);
            left_motor.setPower(1.0);

            Thread.sleep(2000);

            right_motor.setPower(0);
            left_motor.setPower(0);*/

            //Drivechain Methods: Forwards, Backwards, TurnRight, TurnLeft

            Main.Forwards(500);
            Thread.sleep(20000);
            telemetry.update();

        }
    }

    public void Forwards(long Time) throws InterruptedException {
        right_motor.setPower(1.0);
        left_motor.setPower(1.0);
        telemetry.addData("right wheel", right_motor.getPower());
        telemetry.addData("left wheel", left_motor.getPower());
        Thread.sleep(Time);

        right_motor.setPower(0);
        left_motor.setPower(0);
        telemetry.addData("right wheel", right_motor.getPower());
        telemetry.addData("left wheel", left_motor.getPower());;
    }

    public void Backwards(long Time) throws InterruptedException {
        right_motor.setPower(-1.0);
        left_motor.setPower(-1.0);
        telemetry.addData("right wheel", right_motor.getPower());
        telemetry.addData("left wheel", left_motor.getPower());
        Thread.sleep(Time);

        right_motor.setPower(0);
        left_motor.setPower(0);
        telemetry.addData("right wheel", right_motor.getPower());
        telemetry.addData("left wheel", left_motor.getPower());
    }

    public void TurnRight(long Time) throws InterruptedException {
        right_motor.setPower(-1.0);
        left_motor.setPower(1.0);
        telemetry.addData("right wheel", right_motor.getPower());
        telemetry.addData("left wheel", left_motor.getPower());
        Thread.sleep(Time);

        right_motor.setPower(0);
        left_motor.setPower(0);
        telemetry.addData("right wheel", right_motor.getPower());
        telemetry.addData("left wheel", left_motor.getPower());
    }

    public void TurnLeft(long Time) throws InterruptedException {
        right_motor.setPower(1.0);
        left_motor.setPower(-1.0);
        telemetry.addData("right wheel", right_motor.getPower());
        telemetry.addData("left wheel", left_motor.getPower());
        Thread.sleep(Time);

        right_motor.setPower(0);
        left_motor.setPower(0);
        telemetry.addData("right wheel", right_motor.getPower());
        telemetry.addData("left wheel", left_motor.getPower());
    }

}
