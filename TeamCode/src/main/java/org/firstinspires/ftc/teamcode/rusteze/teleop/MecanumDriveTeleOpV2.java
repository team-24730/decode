package org.firstinspires.ftc.teamcode.rusteze.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.rusteze.subsystems.Intake;
import org.firstinspires.ftc.teamcode.rusteze.utility.PIDF;
import org.firstinspires.ftc.teamcode.rusteze.RobotConstants;
import org.firstinspires.ftc.teamcode.rusteze.subsystems.RobotV2;
import org.firstinspires.ftc.teamcode.rusteze.utility.RegressionContainer;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="MecanumDriveTeleOp V2")
public class MecanumDriveTeleOpV2 extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime relocalizationTime = new ElapsedTime();
    double outtakeTarget = 0.0;
    double servoTarget = 0.0;
    boolean intakeToggle = false;
    RobotConstants constants = new RobotConstants();
    RobotConstants.Color teamColor = constants.teamColor;
    int currentID = 24;
    GoBildaPinpointDriver driver;
    private double goalDistance;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private Limelight3A limelight;
    private double lastTx = 0;
    private double lowPassYaw = 0;
    private PIDF limelight_PIDF = new PIDF(0.01, 0.035, 0.0025, 0.25, 0.5);
    private double limelightXPos = 0;
    private double limelightYPos = 0;
    private double limelightAngle = 0;


    @Override
    public void runOpMode() {
        RobotV2 robot = new RobotV2(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(8);
        limelight.start();

        //robot.drivetrain.resetPosition();

        waitForStart();
        runtime.reset();
        while(opModeIsActive()) {

            // GOAL DISTANCE CALCULATION
            if (RobotConstants.teamColor == RobotConstants.Color.RED) {
                goalDistance = Math.sqrt(Math.pow(-65 - robot.drivetrain.getPosition().getX(DistanceUnit.INCH), 2) + Math.pow(65 - robot.drivetrain.getPosition().getY(DistanceUnit.INCH), 2));
            } else {
                goalDistance = Math.sqrt(Math.pow(-65 - robot.drivetrain.getPosition().getX(DistanceUnit.INCH), 2) + Math.pow(-65 - robot.drivetrain.getPosition().getY(DistanceUnit.INCH), 2));
            }


            // ARTIFACT INTAKING CODE //
            if (gamepad1.leftBumperWasPressed() && (robot.getState() == RobotV2.State.IDLE || robot.getState() == RobotV2.State.INTAKE)) {
                if (robot.intake.getState() == Intake.State.IDLE) {
                    robot.setState(RobotV2.State.INTAKE);
                    if (gamepad1.back) {
                        robot.intake.reverse();
                    } else {
                        robot.intake.enable();
                    }
                } else if (robot.intake.getState() == Intake.State.INTAKE || robot.intake.getState() == Intake.State.REVERSE) {
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

            /* // OUTTAKE AND SERVO LIVE ADJUSTMENT CONTROLS
            if (gamepad2.yWasPressed()) {
                outtakeTarget += 100;
            }
            if (gamepad2.xWasPressed()) {
                outtakeTarget += 10;
            }
            if (gamepad2.aWasPressed()) {
                outtakeTarget -= 100;
            }
            if (gamepad2.bWasPressed()) {
                outtakeTarget -= 10;
            }

            if (gamepad2.dpadUpWasPressed()) {
                servoTarget += 0.1;
            }
            if (gamepad2.dpadLeftWasPressed()) {
                servoTarget += 0.01;
            }
            if (gamepad2.dpadDownWasPressed()) {
                servoTarget -= 0.1;
            }
            if (gamepad2.dpadRightWasPressed()) {
                servoTarget -= 0.01;
            } */


            // FLYWHEEL VELOCITY AND HOOD CODE //
            if (robot.getState() == RobotV2.State.IDLE || robot.getState() == RobotV2.State.SHOOTING) {
                if (gamepad1.aWasPressed()) {
                    robot.setState(RobotV2.State.IDLE);
                    robot.intake.disable();
                }
                /*if (gamepad1.xWasPressed()) {
                    robot.setState(RobotV2.State.SHOOTING);
                    robot.outtake.setTarget(outtakeTarget); //    2800
                    robot.outtake.setHood(servoTarget);   //    0.97
                    robot.outtake.useControlSystem = true;
                } */
                if (gamepad1.yWasPressed()) {
                    robot.setState(RobotV2.State.SHOOTING);
                    robot.outtake.useControlSystem = true;
                }
                /*if (gamepad1.bWasPressed()) {
                    robot.setState(RobotV2.State.SHOOTING);
                    robot.outtake.setTarget(4400);
                    robot.outtake.setHood(0.25);
                    robot.outtake.useControlSystem = true;
                }*/
            }
            if (robot.getState() == RobotV2.State.SHOOTING) {
                robot.outtake.setTarget(RegressionContainer.getFlywheelRPM(robot.drivetrain.getGoalDistance(teamColor)));
                robot.outtake.setHood(RegressionContainer.getHoodPosition(robot.drivetrain.getGoalDistance(teamColor)) + 0.1);
            }

            if (robot.getState() == RobotV2.State.IDLE) {
                robot.outtake.setPower(0);
                robot.outtake.useControlSystem = false;
            }

            /* if (robot.getState() == RobotV2.State.SHOOTING) {
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

                robot.drivetrain.setMotorPowers(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x + limelight_PIDF.calculate(lastTx), gamepad1.right_bumper); */

            /*} else*/ if (robot.getState() != RobotV2.State.LIFT) {
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

            /*if (gamepad2.right_bumper && robot.turret.getTargetPosition() < 360) {
                robot.turret.setTargetPosition(robot.turret.getTargetPosition() + 1);
            } else if (gamepad2.left_bumper && robot.turret.getTargetPosition() > 0) {
                robot.turret.setTargetPosition(robot.turret.getTargetPosition() - 1);
            }

            if (gamepad2.a) { robot.turret.setTargetPosition(0); }
            if (gamepad2.y) { robot.turret.setTargetPosition(180); } */

            // PINPOINT TESTING
            double xComponent = -65 - robot.drivetrain.getPosition().getX(DistanceUnit.INCH); // -57
            double yComponent;
            if (RobotConstants.teamColor == RobotConstants.Color.RED) {
                yComponent = 65 - robot.drivetrain.getPosition().getY(DistanceUnit.INCH); // 60
            } else {
                yComponent = -65 - robot.drivetrain.getPosition().getY(DistanceUnit.INCH);
            }
            double goalAngle = -Math.toDegrees(Math.atan2(yComponent, xComponent)) + robot.drivetrain.getPosition().getHeading(AngleUnit.DEGREES);

            if (robot.getState() == RobotV2.State.SHOOTING) {
                robot.turret.setSmartTargetPosition(goalAngle);
            }


            // RELOCALIZATION STUFF
            if (gamepad1.dpad_up && robot.getState() == RobotV2.State.IDLE) {
                robot.setState(RobotV2.State.RELOCALIZING);
                relocalizationTime.reset();
                limelightXPos = -10000; // fake value for geometric mean stuff
                limelightYPos = -10000; // fake value for geometric mean stuff
                limelightAngle = -10000; // fake value for geometric mean stuff
            }

            if (gamepad1.dpad_down && robot.getState() != RobotV2.State.SHOOTING) { // press before relocalization
                robot.turret.setTargetPosition(0);
            }

            if (robot.getState() == RobotV2.State.RELOCALIZING) {
                double pastBias = 0.95;

                if (relocalizationTime.time(TimeUnit.MILLISECONDS) < 1000) {
                    telemetry.addLine("RELOCALIZING");
                    LLResult result = limelight.getLatestResult();
                    if (result != null) {
                        if (result.isValid()) {
                            if (limelightXPos == -10000) {
                                limelightXPos = result.getBotpose().getPosition().x * 39.37;
                            } else {
                                limelightXPos = pastBias * limelightXPos + (1 - pastBias) * result.getBotpose().getPosition().x * 39.37; // geometric mean
                            }
                            if (limelightYPos == -10000) {
                                limelightYPos = result.getBotpose().getPosition().y * 39.37;
                            } else {
                                limelightYPos = 0.95 * limelightYPos + (1 - pastBias) * result.getBotpose().getPosition().y * 39.37; // geometric mean
                            }
                            if (limelightAngle == -10000) {
                                limelightAngle = result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);
                            } else {
                                limelightAngle = 0.95 * limelightAngle + (1 - pastBias) * result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);
                            }

                            telemetry.addData("X", limelightXPos);
                            telemetry.addData("Y", limelightYPos);
                            telemetry.addData("Angle", limelightAngle);
                            telemetry.addData("Distance", Math.sqrt(Math.pow(-55.284868 - limelightXPos, 2) + Math.pow((RobotConstants.teamColor == RobotConstants.Color.BLUE ? -58.834503 : 58.834503) - limelightYPos, 2)));
                        } else {
                            telemetry.addLine("Invalid result");
                        }
                    } else {
                        telemetry.addLine("Null result");
                    }
                } else {
                    if (limelightXPos != -10000 && limelightYPos != -10000 && limelightAngle != -10000) {
                        robot.drivetrain.setPosition(limelightXPos, limelightYPos, limelightAngle);
                    }
                    robot.setState(RobotV2.State.IDLE);
                }
            }


            robot.update();

            telemetry.addData("Current Color", RobotConstants.teamColor);
            telemetry.addData("Current State", robot.getState());
            telemetry.addData("Lift Position", robot.lift.leftLift.getPosition());

            telemetry.addLine("------------------------------\nDEBUG INFORMATION:\n");

            //telemetry.addData("Current Intake State", robot.intake.getState());
            //telemetry.addData("TargetRPM", robot.outtake.targetRPM);
            telemetry.addData("Current Flywheel Velocity", robot.outtake.currentRPM);
            telemetry.addData("Current Turret Position (Degrees)", robot.turret.getPosition());
            //telemetry.addData("Current Turret Position (Ticks)", robot.turret.getEncoderPosition());
            telemetry.addData("Targeted Turret Position", robot.turret.getTargetPosition());
            telemetry.addData("Turret Initialized", RobotConstants.turretInitialized);
            telemetry.addData("Pinpoint X Pos", robot.drivetrain.getPosition().getX(DistanceUnit.INCH));
            telemetry.addData("Pinpoint Y Pos", robot.drivetrain.getPosition().getY(DistanceUnit.INCH));
            telemetry.addData("Pinpoint Angle", robot.drivetrain.getPosition().getHeading(AngleUnit.DEGREES));
            telemetry.addData("Goal Distance", robot.drivetrain.getGoalDistance(teamColor));

            //telemetry.addData("Goal Angle", goalAngle);
            telemetry.update();

            //dashboardTelemetry.addData("Flywheel RPM", robot.outtake.currentRPM);
            //dashboardTelemetry.addData("Target RPM", robot.outtake.targetRPM);
            //dashboardTelemetry.addData("Zero", 0);
            //dashboardTelemetry.addData("Max", 6000);
            //dashboardTelemetry.update();
        }
    }

    // Save turret position to file so it can be reused

}