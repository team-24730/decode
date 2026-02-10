package org.firstinspires.ftc.teamcode.rusteze.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="WheelTest")
public class WheelTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotorEx frontRight;

    @Override
    public void runOpMode() {

        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft" );
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft"  );
        backRight  = hardwareMap.get(DcMotorEx.class, "backRight" );
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");

        frontLeft .setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft  .setDirection(DcMotorSimple.Direction.REVERSE);
        backRight .setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();
        while(opModeIsActive()) {

            if (gamepad1.x) {
                frontLeft.setPower(-gamepad1.left_stick_y);
            }
            if (gamepad1.a) {
                backLeft.setPower(-gamepad1.left_stick_y);
            }
            if (gamepad1.b) {
                backRight.setPower(-gamepad1.left_stick_y);
            }
            if (gamepad1.y) {
                frontRight.setPower(-gamepad1.left_stick_y);
            }

        }
    }
}
