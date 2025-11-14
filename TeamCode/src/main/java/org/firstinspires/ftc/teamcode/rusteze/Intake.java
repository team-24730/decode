package org.firstinspires.ftc.teamcode.rusteze;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    DcMotorEx intake;
    public boolean isEnabled;

    public Intake(HardwareMap hwMap) {
        intake = hwMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power) {
        intake.setPower(power);
    }

    public void disable() {
        setPower(0);
        isEnabled = false;
    }

    public void enable() {
        setPower(1);
        isEnabled = true;
    }
}
