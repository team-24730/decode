package org.firstinspires.ftc.teamcode.rusteze.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;

public final class Drivetrain {
    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotorEx frontRight;
    private GoBildaPinpointDriver pinpoint;

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

        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();
    }

    public void setMotorPowers (double axial, double lateral, double rotation, boolean slowEnabled) {
        double y = driveCorrection(axial);
        double x = driveCorrection(lateral * 1.1) ; // counteract imperfect strafing
        double rx = driveCorrection(rotation * 0.75);

        if (slowEnabled) {
            y = y * 0.2;
            x = x * 0.3;
            rx = rx * 0.4;
        }

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

    public double driveCorrection(double x) {
        return 1.1 * (x * x * Math.signum(x) + 0.15 * x);
    }

    public Pose2D getPosition() {
        return pinpoint.getPosition();
    }

    public void setPosition(double xPos, double yPos, double angle) {pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, xPos , yPos, AngleUnit.DEGREES, angle));}

    public void resetPosition() {
        pinpoint.resetPosAndIMU();
    }

    public void update() {
        pinpoint.update();
    }
}
