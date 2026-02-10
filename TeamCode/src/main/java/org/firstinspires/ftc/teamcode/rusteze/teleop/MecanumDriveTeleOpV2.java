package org.firstinspires.ftc.teamcode.rusteze.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.rusteze.subsystems.Intake;
import org.firstinspires.ftc.teamcode.rusteze.utility.PIDF;
import org.firstinspires.ftc.teamcode.rusteze.RobotConstants;
import org.firstinspires.ftc.teamcode.rusteze.subsystems.RobotV2;

import java.util.List;


@TeleOp(name="MecanumDriveTeleOp V2")
public class MecanumDriveTeleOpV2 extends LinearOpMode {
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

    @Override
    public void runOpMode() {
        RobotV2 robot = new RobotV2(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(8);
        limelight.start();

        waitForStart();
        runtime.reset();
        while(opModeIsActive()) {


            // ARTIFACT INTAKING CODE //
            if (gamepad1.backWasPressed() && robot.getState() == RobotV2.State.IDLE && robot.intake.getState() != Intake.State.INTAKE) {
                robot.setState(RobotV2.State.INTAKE);
                robot.intake.reverse();
            } else if (gamepad1.backWasPressed() && robot.getState() == RobotV2.State.INTAKE && robot.intake.getState() != Intake.State.REVERSE) {
                robot.intake.disable();
                robot.setState(RobotV2.State.IDLE);
            }



            if (gamepad1.leftBumperWasPressed() && (robot.getState() == RobotV2.State.IDLE || robot.getState() == RobotV2.State.INTAKE)) {
                if (robot.intake.getState() == Intake.State.IDLE) {
                    robot.setState(RobotV2.State.INTAKE);
                    robot.intake.enable();
                } else if (robot.intake.getState() == Intake.State.INTAKE) {
                    robot.setState(RobotV2.State.IDLE);
                    robot.intake.disable();
                }
            }



            // ARTIFACT TRANSFER SHOOTING CODE //
            if (robot.getState() == RobotV2.State.SHOOTING) {
                if (gamepad1.left_trigger > 0.2) {
                    robot.intake.enableTransfer();
                } else {
                    robot.intake.disable();
                }
            }

            // FLYWHEEL VELOCITY AND HOOD CODE //
            if (robot.getState() == RobotV2.State.IDLE || robot.getState() == RobotV2.State.SHOOTING) {
                if (gamepad1.aWasPressed()) {
                    robot.setState(RobotV2.State.IDLE);
                    robot.intake.disable();
                }
                if (gamepad1.xWasPressed()) {
                    robot.setState(RobotV2.State.SHOOTING);
                    robot.outtake.setTarget(2800); //3200
                    robot.outtake.setHood(0.97);
                    robot.outtake.useControlSystem = true;
                }
                if (gamepad1.yWasPressed()) {
                    robot.setState(RobotV2.State.SHOOTING);
                    robot.outtake.setTarget(3800);
                    robot.outtake.setHood(0.25);
                    robot.outtake.useControlSystem = true;
                }
                if (gamepad1.bWasPressed()) {
                    robot.setState(RobotV2.State.SHOOTING);
                    robot.outtake.setTarget(4400);
                    robot.outtake.setHood(0.25);
                    robot.outtake.useControlSystem = true;
                }
            }

            if (robot.getState() == RobotV2.State.IDLE) {
                robot.outtake.setPower(0);
                robot.outtake.useControlSystem = false;
            }

            if (robot.getState() == RobotV2.State.SHOOTING) {
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

                robot.drivetrain.setMotorPowers(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x + limelight_PIDF.calculate(lastTx), gamepad1.right_bumper);

            } else if (robot.getState() != RobotV2.State.LIFT) {
                robot.drivetrain.setMotorPowers(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_bumper);
            }

            if (robot.getState() == RobotV2.State.SHOOTING) {
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

            if (gamepad1.left_trigger > 0.2 && gamepad1.right_bumper && gamepad1.start) {
                if (robot.getState() == RobotV2.State.LIFT) {
                    robot.setState(RobotV2.State.IDLE);
                    robot.lift.retract();
                }

            } else if (robot.getState() == RobotV2.State.IDLE && gamepad1.left_trigger > 0.2 && gamepad1.right_bumper) {
                robot.setState(RobotV2.State.LIFT);
                robot.lift.extend();
            }

            // LIFT CODE
            if (robot.getState() == RobotV2.State.LIFT) {
                telemetry.addLine("CURRENTLY LIFTING -- Press Left Bumper, Right Bumper, and Start to unlift");
            }


            // TURRET CODE
            if (gamepad1.right_bumper && gamepad1.right_trigger > 0.2) {
                robot.turret.initializeTurret();
            }

            if (gamepad2.right_bumper && robot.turret.getTargetPosition() < 360) {
                robot.turret.setTargetPosition(robot.turret.getTargetPosition() + 1);
            } else if (gamepad2.left_bumper && robot.turret.getTargetPosition() > 0) {
                robot.turret.setTargetPosition(robot.turret.getTargetPosition() - 1);
            }

            if (gamepad2.a) { robot.turret.setTargetPosition(0); }
            if (gamepad2.y) { robot.turret.setTargetPosition(180); }

            robot.update();

            telemetry.addData("Current Color", RobotConstants.teamColor);
            telemetry.addData("Current State", robot.getState());

            telemetry.addLine("------------------------------\nDEBUG INFORMATION:\n");

            telemetry.addData("Current Intake State", robot.intake.getState());
            telemetry.addData("TargetRPM", robot.outtake.targetRPM);
            telemetry.addData("Lift Position", robot.lift.leftLift.getPosition());
            telemetry.addData("Flywheel Velocity", robot.outtake.currentRPM);
            telemetry.addData("Current Turret Position", robot.turret.getPosition());
            telemetry.addData("Targeted Turret Position", robot.turret.getTargetPosition());
            telemetry.addData("Turret Initialized", RobotConstants.turretInitialized);
            telemetry.update();
        }
    }

    // Save turret position to file so it can be reused

}