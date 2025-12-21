package org.firstinspires.ftc.teamcode.rusteze;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="15 Artifact Red Auto Testing")
public class RedAutoTesting15 extends LinearOpMode {



    public void runOpMode() {

        RobotConstants.teamColor = RobotConstants.Color.RED;

        Robot robot = new Robot(hardwareMap);
        ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        class SpinUpFlywheelClose implements Action {
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    robot.setOuttakeTarget(3350);
                    robot.outtake.setHood(0.9);
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

        class SpinUpFlywheelMedium implements Action {
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    robot.setOuttakeTarget(3850);
                    robot.outtake.setHood(0.25);
                    initialized = true;
                    elapsedTime.reset();
                }
                packet.put("Elapsed Time", elapsedTime.time());

                if (elapsedTime.time() < 1500) {
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


        Pose2d initialPose = new Pose2d(-52, 46, Math.toRadians(125));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder shootPreload = drive.actionBuilder(initialPose)
                /*.afterTime(0, new SpinUpFlywheelClose())
                .afterTime(0.3, new ShootClose())
                .strafeTo(new Vector2d(-38, 26)) // drive to preload shooting position
                .waitSeconds(0.0)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(10, 15, Math.toRadians(90)), Math.toRadians(0)) // get ready to intake at second spike mark
                .afterTime(0, new IntakeOn())
                .strafeTo(new Vector2d(10, 46)) // drive to intake position of second spike mark
                .afterTime(0, new IntakeOff())
                .afterTime(0.5, new SpinUpFlywheelClose())
                .setReversed(true)
                .splineTo(new Vector2d(-4, 22), Math.toRadians(270)) // drive to second spike mark shooting position
                .afterTime(0.85, new ShootClose())
                .strafeToLinearHeading(new Vector2d(-34, 26), Math.toRadians(125)) // shoot second spike mark
                .waitSeconds(0.15)
                .setReversed(true)
                .splineTo(new Vector2d(20, 15), Math.toRadians(-25)) // prepare to open gate and intake
                .setReversed(false)
                .splineTo(new Vector2d(6, 62), Math.toRadians(110), new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 60;
                    }
                }) // open gate and intake
                .afterTime(0, new IntakeOn())
                .waitSeconds(0.1)
                .strafeTo(new Vector2d(20, 64)) // shuffle around a bit to try to intake more
                .waitSeconds(0.75)
                .afterTime(0, new IntakeOff())
                .afterTime(0.6, new SpinUpFlywheelClose())
                .setReversed(true)
                .splineTo(new Vector2d(12, 20), Math.toRadians(270)) // prepare to shoot gate
                .afterTime(0.9, new ShootClose())
                .strafeToLinearHeading(new Vector2d(-34, 26), Math.toRadians(125)) // shoot gate
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(-11, 20), Math.toRadians(105)) // prepare to intake first spike mark
                .afterTime(0, new IntakeOn())
                .strafeToLinearHeading(new Vector2d(-11, 46), Math.toRadians(90)) // intake first spike mark
                .afterTime(0, new IntakeOff())
                .afterTime(0.5, new SpinUpFlywheelClose())
                .afterTime(1.0, new ShootClose())
                .strafeToLinearHeading(new Vector2d(-34, 26), Math.toRadians(125)) // shoot first spike mark
                .waitSeconds(0.25)
                .setReversed(true)
                .splineTo(new Vector2d(45, 10), Math.toRadians(0)) // prepare to intake third spike mark
                .afterTime(0, new IntakeOn())
                .setReversed(false)
                .splineTo(new Vector2d(32, 46), Math.toRadians(90), new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 100;
                    }
                }) // intake third spike mark
                .afterTime(0, new IntakeOff())
                .setReversed(true)
                .splineTo(new Vector2d(45, 10), Math.toRadians(0)) // prepare to shoot third spike mark
                .afterTime(0.5, new SpinUpFlywheelMedium())
                .afterTime(1.25, new ShootClose())
                .setReversed(false)
                .splineTo(new Vector2d(-12, 10), Math.toRadians(135)) // shoot third spike mark
                .waitSeconds(0.1)
                .setReversed(true)
                .strafeTo(new Vector2d(20, 10)); // park */
                .afterTime(0, new SpinUpFlywheelClose())
                .afterTime(0.3, new ShootClose())
                .strafeTo(new Vector2d(-38, 26)) // drive to preload shooting position
                .waitSeconds(0.0)
                .setTangent(0)
                .strafeToLinearHeading(new Vector2d(10, 13), Math.toRadians(90)) // get ready to intake at second spike mark
                .afterTime(0, new IntakeOn())
                .strafeTo(new Vector2d(10, 45)) // drive to intake position of second spike mark
                .afterTime(0, new IntakeOff())
                .afterTime(0.5, new SpinUpFlywheelMedium())
                .afterTime(1.2, new ShootClose())
                .strafeToLinearHeading(new Vector2d(-14, 10), Math.toRadians(135)) // shoot second spike mark
                .waitSeconds(0.4)
                .setReversed(true)
                .splineTo(new Vector2d(20, 15), Math.toRadians(25)) // prepare to open gate and intake
                .setReversed(false)
                .splineTo(new Vector2d(7, 62), Math.toRadians(110), new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 60;
                    }
                }) // open gate and intake
                .afterTime(0, new IntakeOn())
                .waitSeconds(0.1)
                .strafeTo(new Vector2d(20, 64)) // shuffle around a bit to try to intake more
                .waitSeconds(1)
                .afterTime(0, new IntakeOff())
                .afterTime(0.8, new SpinUpFlywheelMedium())
                //.strafeToLinearHeading(new Vector2d(12, 20), Math.toRadians(90)) // prepare to shoot gate
                .strafeTo(new Vector2d(10, 30))
                .afterTime(1.6, new ShootClose())
                .strafeToLinearHeading(new Vector2d(-14, 10), Math.toRadians(135)) // shoot gate
                .waitSeconds(1.5)
                //.strafeToLinearHeading(new Vector2d(-11, 20), Math.toRadians(105)) // prepare to intake first spike mark
                .afterTime(0, new IntakeOn())
                .strafeToLinearHeading(new Vector2d(-11, 54), Math.toRadians(90)) // intake first spike mark
                .afterTime(0, new IntakeOff())
                .afterTime(0.5, new SpinUpFlywheelClose())
                .afterTime(1.0, new ShootClose())
                .strafeToLinearHeading(new Vector2d(-34, 26), Math.toRadians(125)) // shoot first spike mark
                .waitSeconds(0.4)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(10, 15), Math.toRadians(90)) // prepare to intake third spike mark
                .afterTime(0, new IntakeOn())
                .setReversed(false)
                .splineTo(new Vector2d(32, 54), Math.toRadians(90), new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 500;
                    }
                }) // intake third spike mark
                .afterTime(0, new IntakeOff())
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(5, 25), Math.toRadians(90)) // prepare to shoot third spike mark
                .afterTime(0.5, new SpinUpFlywheelMedium())
                .afterTime(1.25, new ShootClose())
                .strafeToLinearHeading(new Vector2d(-14, 10), Math.toRadians(135)) // shoot third spike mark
                .waitSeconds(1.0)
                .strafeTo(new Vector2d(7, 10)); // park

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


