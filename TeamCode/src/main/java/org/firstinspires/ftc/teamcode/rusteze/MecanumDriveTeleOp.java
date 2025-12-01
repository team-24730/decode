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

            /*

            if (!intakeToggle) {
                if (gamepad1.x) {
                    robot.setOuttakeTarget(3000);
                    robot.outtake.setHood(0.97);
                }

                if (gamepad1.y) {
                    robot.setOuttakeTarget(4100);
                    robot.outtake.setHood(0.3);
                }

                if (gamepad1.b) {
                    robot.setOuttakeTarget(4750);
                    robot.outtake.setHood(0.25);
                }

                if (gamepad1.a) {
                    robot.setOuttakeTarget(0);
                }
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

            */


            if (gamepad1.leftBumperWasPressed()) {
                if (robot.getState() == Robot.State.IDLE) {
                    robot.setState(Robot.State.INTAKE);
                } else if (robot.getState() == Robot.State.INTAKE) {
                    robot.setState(Robot.State.INTAKE_REVERSE);
                }
            }

            if (robot.getState() == Robot.State.IDLE || robot.getState() == Robot.State.OUTTAKE_IDLE) {
                if (gamepad1.aWasPressed()) {
                    robot.setState(Robot.State.IDLE);
                }
                if (gamepad1.xWasPressed()) {
                    robot.setState(Robot.State.OUTTAKE_IDLE);
                    robot.setOuttakeTarget(3000);
                    robot.outtake.setHood(0.97);
                }
                if (gamepad1.yWasPressed()) {
                    robot.setState(Robot.State.OUTTAKE_IDLE);
                    robot.setOuttakeTarget(4100);
                    robot.outtake.setHood(0.3);
                }
                if (gamepad1.bWasPressed()) {
                    robot.setState(Robot.State.OUTTAKE_IDLE);
                    robot.setOuttakeTarget(4750);
                    robot.outtake.setHood(0.25);
                }
            }

            if (gamepad1.left_trigger > 0.2 && robot.getState() == Robot.State.OUTTAKE_IDLE) {
                robot.setState(Robot.State.OUTTAKE_SHOOT);
            }

            if (robot.getState() == Robot.State.OUTTAKE_SHOOT && !(gamepad1.left_trigger > 0.2)) {
                robot.setState(Robot.State.OUTTAKE_IDLE);
            }

            robot.setDrivetrainTarget(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_bumper);

            robot.update();

            telemetry.addData("TargetRPM", robot.outtake.targetRPM);
            telemetry.addData("Servo Pos", servoDebug);
            telemetry.addData("Current State", robot.getState());
            telemetry.update();

        }
    }
}