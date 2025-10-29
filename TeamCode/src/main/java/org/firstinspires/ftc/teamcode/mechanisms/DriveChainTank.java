package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveChainTank {
    private DcMotor leftDrive, rightDrive;

    public void init(HardwareMap hardwareMap) {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    public void moveRobot(double x, double yaw) {

        double leftPower    = x - yaw;
        double rightPower   = x + yaw;

        //normalize wheel powers just below 1
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max >1.0) {
            leftPower /= max;
            rightPower /= max;
        }
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }
}
