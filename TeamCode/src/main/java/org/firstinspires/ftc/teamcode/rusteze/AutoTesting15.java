package org.firstinspires.ftc.teamcode.rusteze;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="15 Artifact Blue Auto Testing")
public class AutoTesting15 extends LinearOpMode {


    public void runOpMode() {

        Robot robot = new Robot(hardwareMap);
        ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        class SpinUpFlywheelClose implements Action {
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    robot.setOuttakeTarget(3200);
                    robot.outtake.setHood(0.97);
                    initialized = true;
                    elapsedTime.reset();
                }
                packet.put("Elapsed Time", elapsedTime.time());

                if (elapsedTime.time() < 1000) {
                    robot.outtake.update(); // allow the PID controller a second to spin up the flywheel
                    return true;
                } else {
                    return false;
                }
            }
        }

        class SpinUpFlywheelFar implements Action {
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    robot.setOuttakeTarget(5000);
                    robot.outtake.setHood(0.25);
                    robot.outtake.update();
                    initialized = true;
                    elapsedTime.reset();
                }
                packet.put("Elapsed Time", elapsedTime.time());

                if (elapsedTime.time() < 1500) {
                    robot.outtake.update(); // allow the PID controller a second and a half to spin up the flywheel
                    return true;
                } else {
                    return false;
                }
            }
        }

        class ShootClose implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    robot.intake.setPower(1);
                    robot.transfer.setPower(1);
                    initialized = true;
                    elapsedTime.reset();
                }

                packet.put("Elapsed Time", elapsedTime.time());
                if (elapsedTime.time() < 1000) {
                    robot.outtake.update(); // continue updating the outtake so the PID controller can keep flywheel speed constant as artifacts move through the shooter
                    return true;
                } else {
                    robot.intake.setPower(0);
                    robot.transfer.setPower(0);
                    robot.setOuttakeRawPower(0);
                    robot.outtake.update();
                    return false;
                }
            }
        }

        class ShootFar implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    robot.intake.setPower(1);
                    robot.transfer.setPower(0.75);
                    initialized = true;
                    elapsedTime.reset();
                }

                packet.put("Elapsed Time", elapsedTime.time());
                if (elapsedTime.time() < 1500) {
                    robot.outtake.update(); // continue updating the outtake so the PID controller can keep flywheel speed constant as artifacts move through the shooter
                    return true;
                } else {
                    robot.intake.setPower(0);
                    robot.transfer.setPower(0);
                    robot.setOuttakeRawPower(0);
                    robot.outtake.update();
                    return false;
                }
            }
        }

        class IntakeOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.intake.setPower(1);
                robot.transfer.setPower(1);
                packet.put("Intake Enabled", true);
                return false;
            }
        }

        class IntakeOff implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    robot.intake.setPower(-0.1);
                    robot.transfer.setPower(-0.5);
                    robot.setOuttakeRawPower(-0.35);
                    robot.outtake.update();
                    initialized = true;
                    elapsedTime.reset();
                }

                packet.put("Elapsed Time", elapsedTime.time());
                if (elapsedTime.time() < 500) {
                    return true;
                } else {
                    robot.intake.setPower(0);
                    robot.transfer.setPower(0);
                    robot.setOuttakeRawPower(0);
                    robot.outtake.update();
                    return false;
                }
            }
        }


        Pose2d initialPose = new Pose2d(-52, -46, Math.toRadians(235));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder shootPreload = drive.actionBuilder(initialPose)
                .afterTime(0, new SpinUpFlywheelClose())
                .afterTime(0.7, new ShootClose())
                .strafeTo(new Vector2d(-38, -26)) // drive to preload shooting position
                .waitSeconds(0.6)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(10, -12, Math.toRadians(270)), Math.toRadians(0)) // get ready to intake at second spike mark
                .afterTime(0, new IntakeOn())
                .strafeTo(new Vector2d(18, -44)) // drive to intake position of second spike mark
                .afterTime(0, new IntakeOff())
                .afterTime(0.75, new SpinUpFlywheelClose())
                .setReversed(true)
                .splineTo(new Vector2d(-4, -12), Math.toRadians(90)) // drive to second spike mark shooting position
                .afterTime(0.75, new ShootClose())
                .strafeToLinearHeading(new Vector2d(-34, -26), Math.toRadians(235)) // shoot second spike mark
                .waitSeconds(0.6)
                .setReversed(true)
                .splineTo(new Vector2d(24, -14), Math.toRadians(25)) // prepare to open gate and intake
                .setReversed(false)
                .splineTo(new Vector2d(10, -54), Math.toRadians(244)) // open gate and intake //240
                .afterTime(0, new IntakeOn())
                .waitSeconds(1.5)
                .afterTime(0, new IntakeOff())
                .afterTime(0.75, new SpinUpFlywheelClose())
                .setReversed(true)
                .splineTo(new Vector2d(12, -10), Math.toRadians(90)) // prepare to shoot gate
                .afterTime(0.9, new ShootClose())
                .strafeToLinearHeading(new Vector2d(-34, -26), Math.toRadians(235)) // shoot gate
                .waitSeconds(0.6)
                .strafeToLinearHeading(new Vector2d(-10, -10), Math.toRadians(255)) // prepare to intake first spike mark
                .afterTime(0, new IntakeOn())
                .strafeToLinearHeading(new Vector2d(-10, -46), Math.toRadians(270)) // intake first spike mark
                .afterTime(0, new IntakeOff())
                .afterTime(0.75, new SpinUpFlywheelClose())
                .afterTime(1.15, new ShootClose())
                .strafeToLinearHeading(new Vector2d(-34, -26), Math.toRadians(235)) // shoot first spike mark
                .waitSeconds(0.6)
                .setReversed(true)
                .splineTo(new Vector2d(50, -20), Math.toRadians(0)) // prepare to intake third spike mark
                .afterTime(0, new IntakeOn())
                .setReversed(false)
                .splineTo(new Vector2d(34, -50), Math.toRadians(270)) // intake third spike mark
                .afterTime(0, new IntakeOff())
                .afterTime(0.75, new SpinUpFlywheelClose())
                .setReversed(true)
                .splineTo(new Vector2d(-15, -15), Math.toRadians(135)) // prepare to shoot third spike mark
                .strafeToLinearHeading(new Vector2d(-34, -26), Math.toRadians(235)) // shoot third spike mark
                .afterTime(0, new ShootClose())
                .waitSeconds(0.6);

        waitForStart();
        while(opModeIsActive()) {


            Actions.runBlocking(
                    new SequentialAction(
                            shootPreload.build()
                    )
            );

            requestOpModeStop();
        }

    }
}


