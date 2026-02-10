package org.firstinspires.ftc.teamcode.rusteze.autos.archived;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.rusteze.subsystems.Robot;
import org.firstinspires.ftc.teamcode.rusteze.RobotConstants;

@Autonomous(name="15 Artifact Blue Auto")
@Disabled
public class BlueAuto15 extends LinearOpMode {


    public void runOpMode() {

        RobotConstants.teamColor = RobotConstants.Color.BLUE;

        Robot robot = new Robot(hardwareMap);
        ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        class SpinUpFlywheelClose implements Action {
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    robot.setOuttakeTarget(3500);
                    robot.outtake.setHood(0.65);
                    initialized = true;
                    elapsedTime.reset();
                }
                packet.put("Elapsed Time", elapsedTime.time());

                if (elapsedTime.time() < 500) {
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
                    robot.setOuttakeTarget(4750);
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
                .afterTime(0, new SpinUpFlywheelClose()) // spin up flywheel for close shooting
                .setReversed(true)
                .splineTo(new Vector2d(-26, -12), Math.toRadians(55)); // drive to shooting position for preloads

        TrajectoryActionBuilder intakeFirstSpike = shootPreload.endTrajectory().fresh()
                .setReversed(false)
                .splineTo(new Vector2d(-8, -40), Math.toRadians(270)); // intake at first spike mark

        TrajectoryActionBuilder shootFirstSpike = intakeFirstSpike.endTrajectory().fresh()
                .afterTime(0.6, new SpinUpFlywheelClose()) // spin up flywheel for close shooting
                .setReversed(true)
                .splineTo(new Vector2d(-30, -16), Math.toRadians(55)); // go to close shoot position

        TrajectoryActionBuilder openGate = shootFirstSpike.endTrajectory().fresh()
                .setReversed(false)
                .splineTo(new Vector2d(-4, -53), Math.toRadians(270)) // open gate
                .waitSeconds(1) // wait for a second so the gate can open
                .setReversed(true)
                .splineTo(new Vector2d(3, -3), Math.toRadians(90)); // prepare to intake at second spike mark

        TrajectoryActionBuilder intakeSecondSpike = openGate.endTrajectory().fresh()
                .setReversed(false)
                .splineTo(new Vector2d(13, -34), Math.toRadians(270)); // intake at second spike mark

        TrajectoryActionBuilder shootSecondSpike = intakeSecondSpike.endTrajectory().fresh()
                .afterTime(0.6, new SpinUpFlywheelClose()) // spin up flywheel for close shooting
                .setReversed(false)
                .setReversed(true)
                .splineTo(new Vector2d(-30, -16), Math.toRadians(55)); // go to close shoot position

        TrajectoryActionBuilder intakeThirdSpikeBack = shootSecondSpike.endTrajectory().fresh()
                .turnTo(Math.toRadians(210)) // turn so there is no interference with the opposite side
                .setReversed(true)
                .splineTo(new Vector2d(38, -10), Math.toRadians(45)); // prepare to intake at third spike mark

        TrajectoryActionBuilder intakeThirdSpike = intakeThirdSpikeBack.endTrajectory().fresh()
                .setReversed(false)
                .splineTo(new Vector2d(38, -32), Math.toRadians(270)); // intake at third spike mark

        TrajectoryActionBuilder shootThirdSpike = intakeThirdSpike.endTrajectory().fresh()
                .afterTime(0.6, new SpinUpFlywheelFar()) // spin up flywheel for far shooting
                .setReversed(true)
                .splineTo(new Vector2d(56, -14), Math.toRadians(20)); // go to far shoot position

        TrajectoryActionBuilder intakeHPZone = shootThirdSpike.endTrajectory().fresh()
                .setReversed(false)
                .splineTo(new Vector2d(62, -46), Math.toRadians(270)); // intake at hp zone

        TrajectoryActionBuilder shootHPZone = intakeHPZone.endTrajectory().fresh()
                .afterTime(0.6, new SpinUpFlywheelFar()) // spin up flywheel for far shooting
                .setReversed(true)
                .splineTo(new Vector2d(54, -14), Math.toRadians(22)); // go to far shoot position




        // .strafeToLinearHeading(new Vector2d(5, 20), 90);

        waitForStart();
        while(opModeIsActive()) {


            Actions.runBlocking(
                    new SequentialAction(
                            shootPreload.build(),
                            new ShootClose(),

                            new IntakeOn(),
                            intakeFirstSpike.build(),
                            new IntakeOff(),
                            shootFirstSpike.build(),
                            new ShootClose(),

                            openGate.build(),

                            new IntakeOn(),
                            intakeSecondSpike.build(),
                            new IntakeOff(),
                            shootSecondSpike.build(),
                            new ShootClose(),

                            intakeThirdSpikeBack.build(),
                            new IntakeOn(),
                            intakeThirdSpike.build(),
                            new IntakeOff(),
                            shootThirdSpike.build(),
                            new ShootFar(),

                            new IntakeOn(),
                            intakeHPZone.build(),
                            new IntakeOff(),
                            shootHPZone.build(),
                            new ShootFar()

                    )
            );

            requestOpModeStop();
        }

    }
}


