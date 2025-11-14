package org.firstinspires.ftc.teamcode.rusteze;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Transfer {
    public DcMotorEx transfer;
    public boolean isEnabled = false;

    public Transfer(HardwareMap hwMap) {
        transfer = hwMap.get(DcMotorEx.class, "transfer");
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power) {
        transfer.setPower(power);
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
