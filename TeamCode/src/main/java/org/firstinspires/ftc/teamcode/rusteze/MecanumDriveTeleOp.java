package org.firstinspires.ftc.teamcode.rusteze;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="MecanumDriveTeleOp")
public class MecanumDriveTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    double outtakeTarget = 0.0;
    boolean intakeToggle = false;

    double servoDebug = 0.25;

    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap);

        waitForStart();
        runtime.reset();
        while(opModeIsActive()) {

            robot.drivetrain.setMotorPowers(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if (gamepad1.x) {
                robot.setOuttakeTarget(3000);
                robot.outtake.setHood(0.97);
            }

            if (gamepad1.y) {
                robot.setOuttakeTarget(4000);
                robot.outtake.setHood(0.3);
            }

            if (gamepad1.b) {
                robot.setOuttakeTarget(4750);
                robot.outtake.setHood(0.25);
            }

            if (gamepad1.a) {
                robot.setOuttakeTarget(0);
            }

            if (gamepad1.leftBumperWasPressed()) {
                intakeToggle = !intakeToggle;
            }

            if (gamepad1.right_bumper) {
                robot.transfer.setPower(1);
            } else if (!robot.dontTurnOffTransfer) {
                robot.transfer.setPower(0);
            }

            robot.intakeRunning = intakeToggle;

            robot.update();

            telemetry.addData("TargetRPM", robot.outtake.targetRPM);
            telemetry.addData("Servo Pos", servoDebug);
            telemetry.update();

        }
    }
}