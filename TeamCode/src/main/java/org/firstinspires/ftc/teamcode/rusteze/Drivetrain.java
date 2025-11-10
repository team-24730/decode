package org.firstinspires.ftc.teamcode.rusteze;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class Drivetrain {
    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotorEx frontRight;

    public Drivetrain(HardwareMap hwMap) {
        frontLeft  = hwMap.get(DcMotorEx.class, "frontLeft" );
        backLeft   = hwMap.get(DcMotorEx.class, "backLeft"  );
        backRight  = hwMap.get(DcMotorEx.class, "backRight" );
        frontRight = hwMap.get(DcMotorEx.class, "frontRight");

        frontLeft .setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft  .setDirection(DcMotorSimple.Direction.REVERSE);
        backRight .setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMotorPowers (double axial, double lateral, double rotation) {
        double y = axial;
        double x = lateral * 1.1; // counteract imperfect strafing
        double rx = rotation;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y + x - rx) / denominator;
        double backRightPower = (y - x - rx) / denominator;

        frontLeft .setPower(frontLeftPower);
        backLeft  .setPower(backLeftPower);
        backRight .setPower(frontRightPower);
        frontRight.setPower(backRightPower);
    }
}
