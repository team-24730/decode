package org.firstinspires.ftc.teamcode.rusteze;

import android.icu.text.RelativeDateTimeFormatter;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Shooter Test")
public class ShooterTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx flywheelLeft;
    private DcMotorEx flywheelRight;
    private CRServo hood;

    @Override
    public void runOpMode() {

        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        hood = hardwareMap.get(CRServo.class, "hood");

        flywheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();
        runtime.reset();
        while(opModeIsActive()) {

            flywheelLeft.setPower(gamepad1.right_trigger);
            flywheelRight.setPower(gamepad1.right_trigger);

            hood.setPower(-gamepad1.left_stick_y);

        }
    }
}
