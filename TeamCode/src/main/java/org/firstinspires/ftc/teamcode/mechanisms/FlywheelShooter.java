package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FlywheelShooter {
    private DcMotorEx shooter;

    private double ticksPerRot;
    public void init(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ticksPerRot = shooter.getMotorType().getTicksPerRev();
    }

    public void start(double power) {
        shooter.setVelocity(power);
    }

    public void kill() {
        shooter.setVelocity(0);
    }

}
