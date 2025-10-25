package org.firstinspires.ftc.teamcode.rusteze;

import android.icu.text.RelativeDateTimeFormatter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name="Shooter Test")
public class ShooterTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx flywheelLeft;
    private DcMotorEx flywheelRight;
    private CRServo hood;

    private final int TICKS_PER_REVOLUTION = 28;

    RobotConstants constants = new RobotConstants();



    @Override
    public void runOpMode() {

        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        hood = hardwareMap.get(CRServo.class, "hood");

        flywheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        runtime.reset();
        while(opModeIsActive()) {

            flywheelLeft.setPower(gamepad1.right_trigger);
            flywheelRight.setPower(gamepad1.right_trigger);

            // hood.setPower(-gamepad1.left_stick_y);

            FtcDashboard dashboard = FtcDashboard.getInstance();
            Telemetry dashboardTelemetry = dashboard.getTelemetry();

            dashboardTelemetry.addData("Target RPM", constants.getTargetRpm());
            dashboardTelemetry.addData("Current Power", flywheelRight.getPower());
            dashboardTelemetry.addData("Current TPS", flywheelRight.getVelocity());
            dashboardTelemetry.addData("Current RPM", flywheelRight.getVelocity() / TICKS_PER_REVOLUTION * 60);

            telemetry.addData("Target RPM", constants.getTargetRpm());
            telemetry.addData("Current Power", flywheelRight.getPower());
            telemetry.addData("Current TPS", flywheelRight.getVelocity());
            telemetry.addData("Current RPM", flywheelRight.getVelocity() / TICKS_PER_REVOLUTION * 60);

            telemetry.update();
            dashboardTelemetry.update();

        }
    }
}
