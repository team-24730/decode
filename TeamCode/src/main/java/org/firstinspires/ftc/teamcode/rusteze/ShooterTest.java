package org.firstinspires.ftc.teamcode.rusteze;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

            // hood.setPower(-gamepad1.left_stick_y);


            // FLYWHEEL PID

            double targetRPM = constants.getTargetRpm();
            double currentRPM = flywheelRight.getVelocity() / TICKS_PER_REVOLUTION * 60; // Convert ticks per second to revolutions per minute
            double errorRPM = targetRPM - currentRPM;
            double kV = constants.getkV();
            double kP = constants.getkP();

            double vComponent = targetRPM * kV; // Calculate feedforward (SVA) V component
            double pComponent = errorRPM * kP; // Calculate feedback (PID) P component

            double flywheelPower = Math.min(1, Math.max(    vComponent + pComponent    , 0)); // Calculate and clamp flywheel power between 0 and 1

            flywheelLeft.setPower(flywheelPower);
            flywheelRight.setPower(flywheelPower);


            // TELEMETRY

            FtcDashboard dashboard = FtcDashboard.getInstance();
            Telemetry dashboardTelemetry = dashboard.getTelemetry();

            dashboardTelemetry.addData("Target RPM", targetRPM);
            dashboardTelemetry.addData("Current Power", flywheelPower);
            dashboardTelemetry.addData("Current RPM", currentRPM);
            dashboardTelemetry.addData("Lowest RPM", 0);
            dashboardTelemetry.addData("Highest RPM", 6000);

            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current Power", flywheelPower);
            telemetry.addData("Current RPM", currentRPM);

            telemetry.update();
            dashboardTelemetry.update();

        }
    }
}
