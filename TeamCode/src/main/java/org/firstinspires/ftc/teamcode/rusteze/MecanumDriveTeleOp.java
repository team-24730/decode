package org.firstinspires.ftc.teamcode.rusteze;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.PlaceholderCalibratedAspectRatioMismatch;


@TeleOp(name="MecanumDriveTeleOp")
public class MecanumDriveTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    double outtakeTarget = 0.0;
    boolean intakeToggle = false;

    private Limelight3A limelight;
    private double lastTx = 0;
    private double lowPassYaw = 0;
    private double limelight_k_P = 0.045;
    private double limelight_k_S = 0.3;
    private PIDF limelight_PIDF = new PIDF(0.01, 0.035, 0.0025, 0.25, 0.5);

    double servoDebug = 0.25;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(8);
        limelight.start();

        if (gamepad2.right_bumper) {
            limelight_k_P = 100;
        }

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


            if (robot.getState() == Robot.State.OUTTAKE_IDLE || robot.getState() == Robot.State.OUTTAKE_SHOOT) {
                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    if (result.isValid()) {
                        double k = 0.02;

                        Pose3D botpose = result.getBotpose();
                        telemetry.addData("tx", result.getTx());
                        lowPassYaw = Math.abs(result.getBotpose().getOrientation().getYaw()) * k     +     (1 - k) * lowPassYaw;
                        lastTx = result.getTx() + (lowPassYaw - 90)/90 * 11 - 5;
                        telemetry.addData("Angle Adjustment", (lowPassYaw - 90)/90 * 11 - 5);
                        telemetry.addData("Low pass yaw", lowPassYaw);
                        telemetry.addData("ty", result.getTy());
                        telemetry.addData("Botpose", botpose.toString());
                    } else {
                        telemetry.addLine("Invalid result");
                        lastTx = 0;
                        lowPassYaw = 135;
                    }
                } else {
                    telemetry.addLine("Null result");
                    lastTx = 0;
                    lowPassYaw = 135;
                }

                robot.setDrivetrainTarget(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x + limelight_PIDF.calculate(lastTx), gamepad1.right_bumper);

            } else {
                robot.setDrivetrainTarget(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_bumper);
            }



            robot.update();

            telemetry.addData("TargetRPM", robot.outtake.targetRPM);
            telemetry.addData("Servo Pos", servoDebug);
            telemetry.addData("Current State", robot.getState());
            telemetry.update();

        }
    }
}