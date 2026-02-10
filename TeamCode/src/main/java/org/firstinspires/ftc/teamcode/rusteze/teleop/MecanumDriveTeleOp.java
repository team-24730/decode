package org.firstinspires.ftc.teamcode.rusteze.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.rusteze.utility.PIDF;
import org.firstinspires.ftc.teamcode.rusteze.subsystems.Robot;
import org.firstinspires.ftc.teamcode.rusteze.RobotConstants;

import java.util.List;


@TeleOp(name="MecanumDriveTeleOp")
public class MecanumDriveTeleOp extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    double outtakeTarget = 0.0;
    boolean intakeToggle = false;
    RobotConstants constants = new RobotConstants();
    RobotConstants.Color teamColor = constants.teamColor;
    int currentID = 24;

    private Limelight3A limelight;
    private double lastTx = 0;
    private double lowPassYaw = 0;
    private PIDF limelight_PIDF = new PIDF(0.01, 0.035, 0.0025, 0.25, 0.5);
    double aprilTagDistance = 0;

    double servoDebug = 0.25;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(8);
        limelight.start();

        waitForStart();
        runtime.reset();
        while(opModeIsActive()) {

            if (gamepad1.leftBumperWasPressed()) {
                if (robot.getState() == Robot.State.IDLE) {
                    robot.outtake.setHood(0.97);
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
                    robot.setTransferSpeed(1);
                    robot.setState(Robot.State.OUTTAKE_IDLE);
                    robot.setOuttakeTarget(3200);
                    robot.outtake.setHood(0.97);
                }
                if (gamepad1.yWasPressed()) {
                    robot.setTransferSpeed(1);
                    robot.setState(Robot.State.OUTTAKE_IDLE);
                    robot.setOuttakeTarget(4000);
                    robot.outtake.setHood(0.4);
                }
                if (gamepad1.bWasPressed()) {
                    robot.setTransferSpeed(0.8);
                    robot.setState(Robot.State.OUTTAKE_IDLE);
                    robot.setOuttakeTarget(4900);
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
                        if (lowPassYaw == 0) {
                            lowPassYaw = Math.abs(result.getBotpose().getOrientation().getYaw());
                        } else {
                            lowPassYaw = Math.abs(result.getBotpose().getOrientation().getYaw()) * k   +  (1 - k) * lowPassYaw;
                        }

                        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                        for (LLResultTypes.FiducialResult fiducial : fiducials) {
                            currentID = fiducial.getFiducialId();
                        }
                        telemetry.addData("Current ID", currentID);

                        telemetry.addData("Low Pass Yaw", lowPassYaw);


                        if (currentID == 24 && teamColor == RobotConstants.Color.RED) {
                            lastTx = result.getTx() + ((lowPassYaw - 90)/90 - 0.5) * -4;
                            telemetry.addData("Angle Adjustment", ((lowPassYaw - 90)/90 - 0.5) * -4);
                        }

                        if (currentID == 20 && teamColor == RobotConstants.Color.BLUE) {
                            lastTx = result.getTx() + ((lowPassYaw - 90)/90 - 0.5) * -4;
                            telemetry.addData("Angle Adjustment", ((lowPassYaw - 90)/90 - 0.5) * -4);
                        }

                    } else {
                        telemetry.addLine("Invalid result");
                        lastTx = 0;
                        lowPassYaw = 0;
                    }
                } else {
                    telemetry.addLine("Null result");
                    lastTx = 0;
                    lowPassYaw = 0;
                }

                robot.setDrivetrainTarget(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x + limelight_PIDF.calculate(lastTx), gamepad1.right_bumper);

            } else {
                robot.setDrivetrainTarget(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_bumper);
            }

            if (robot.getState() == Robot.State.OUTTAKE_IDLE || robot.getState() == Robot.State.OUTTAKE_SHOOT) {
                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    if (result.isValid()) {
                        double xPos = result.getBotpose().getPosition().x * 39.37; // convert meters to inches
                        double yPos = result.getBotpose().getPosition().y * 39.37; // convert meters to inches
                        telemetry.addData("X", xPos);
                        telemetry.addData("Y", yPos);
                        telemetry.addData("Distance", Math.sqrt(Math.pow(-55.284868 - xPos, 2) + Math.pow((RobotConstants.teamColor == RobotConstants.Color.BLUE ? -58.834503 : 58.834503) - yPos, 2)));
                    } else {
                        telemetry.addLine("Invalid result");
                    }
                } else {
                    telemetry.addLine("Null result");
                }
            }


            if (robot.getState() == Robot.State.IDLE) {
                if (gamepad1.a && gamepad1.right_bumper && gamepad1.left_trigger > 0.2) {
                    robot.setState(Robot.State.LIFT);
                    robot.lift.extend();
                }
            }

            /*if (robot.getState().equals(Robot.State.IDLE) && gamepad1.back && gamepad1.a) {
                robot.lift.prime();
            } */

            if (robot.getState() == Robot.State.LIFT) {
                if (gamepad1.a && gamepad1.right_bumper && gamepad1.right_trigger > 0.2) {
                    robot.lift.retract();
                    robot.setState(Robot.State.IDLE);
                }
            }




            robot.update();

            telemetry.addData("Lift pos", robot.lift.leftLift.getPosition());
            telemetry.addData("Current Color", RobotConstants.teamColor);
            telemetry.addData("TargetRPM", robot.outtake.targetRPM);
            telemetry.addData("Servo Pos", servoDebug);
            telemetry.addData("Current State", robot.getState());
            telemetry.update();
        }
    }
}