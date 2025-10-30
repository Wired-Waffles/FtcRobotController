package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RubberBandIntake {
    private DcMotor intake;

    private double ticksPerRot;
    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ticksPerRot = intake.getMotorType().getTicksPerRev();
    }

    public void intakePower(double power) {
        intake.setPower(power);
    }

    public void eject() throws InterruptedException {
        intake.setPower(-0.8);
        wait(1500);
        intake.setPower(0);
        wait(100);
    }

    public double getTotalRots (){
        double gearRatio = 25 / 3;
        return intake.getCurrentPosition() / ticksPerRot * gearRatio;
    }
}
