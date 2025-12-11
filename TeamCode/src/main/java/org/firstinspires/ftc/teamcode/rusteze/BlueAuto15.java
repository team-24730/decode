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

@Autonomous(name="15 Artifact Blue Auto")
public class BlueAuto15 extends LinearOpMode {


    public void runOpMode() {

        Robot robot = new Robot(hardwareMap);
        ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        class OuttakeClose implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    robot.setOuttakeTarget(3200);
                    robot.outtake.setHood(0.97);
                    initialized = true;
                    elapsedTime.reset();
                }

                packet.put("Elapsed Time", elapsedTime.time());
                if (elapsedTime.time() < 1500) {
                    if (elapsedTime.time() > 250) {
                        robot.transfer.setPower(1);
                    }
                    robot.outtake.update();
                    return true;
                } else {
                    robot.setOuttakeTarget(0);
                    robot.outtake.update();
                    robot.transfer.setPower(0);
                    return false;
                }
            }
        }

        class OuttakeBackwards implements Action {
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

        class IntakeOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.intake.setPower(1);
                robot.transfer.setPower(1);
                packet.put("Transfer", true);
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
                    initialized = true;
                    elapsedTime.reset();
                }

                packet.put("Elapsed Time", elapsedTime.time());
                if (elapsedTime.time() < 500) {
                    return true;
                } else {
                    robot.intake.setPower(0);
                    robot.transfer.setPower(0);
                    return false;
                }
            }
        }


        Pose2d initialPose = new Pose2d(65.0147, -37.6669, Math.toRadians(-179.808));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder farShootPreload = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(63.8587, -39.1769), Math.toRadians(-165.4041));

        TrajectoryActionBuilder wallIntakeHPPre = drive.actionBuilder(new Pose2d(63.8587, -39.1769, Math.toRadians(-165.4041)))
                .splineTo(new Vector2d(49.8117, -61.1149), Math.toRadians(-11.579));

        TrajectoryActionBuilder wallIntakeHPPost = drive.actionBuilder(new Pose2d(49.8117, -61.1149, Math.toRadians(-11.579)))
                .splineTo(new Vector2d(62.2813, -62.2142), Math.toRadians(-0.8206));

        TrajectoryActionBuilder mediumShoot1 = drive.actionBuilder(new Pose2d(62.2813, -62.2142, Math.toRadians(-0.8206)))
                .setReversed(true)
                .splineTo(new Vector2d(11.035, -14.9189), Math.toRadians(-137.9632));

        TrajectoryActionBuilder intakePreplace1 = drive.actionBuilder(new Pose2d(11.035, -14.9189, Math.toRadians(-137.9632)))
                .splineTo(new Vector2d(1.4807, -57.2865), Math.toRadians(-90));

        TrajectoryActionBuilder gateOpenBack = drive.actionBuilder(new Pose2d(1.4807, -57.2865, Math.toRadians(-90)))
                .setReversed(true)
                .splineTo(new Vector2d(11.035, -14.9189), Math.toRadians(-137.9632));

        TrajectoryActionBuilder gateOpenForwards = drive.actionBuilder(new Pose2d(11.035, -14.9189, Math.toRadians(-137.9632)))
                .splineTo(new Vector2d(5.316, -41.0164), Math.toRadians(-90));

        TrajectoryActionBuilder mediumShoot2 = drive.actionBuilder(new Pose2d(5.316, -41.0164, Math.toRadians(-90)))
                .setReversed(true)
                .splineTo(new Vector2d(11.035, -14.9189), Math.toRadians(-137.9632));

        TrajectoryActionBuilder intakePreplace2 = drive.actionBuilder(new Pose2d(11.035, -14.9189, Math.toRadians(-137.9632)))
                .splineTo(new Vector2d(7.5179, -61.8704), Math.toRadians(-90));

        TrajectoryActionBuilder mediumShoot3 = drive.actionBuilder(new Pose2d(7.5179, -61.8704, Math.toRadians(-90)))
                .setReversed(true)
                .splineTo(new Vector2d(11.035, -14.9189), Math.toRadians(-137.9632));

        TrajectoryActionBuilder intakePreplace3 = drive.actionBuilder(new Pose2d(11.035, -14.9189, Math.toRadians(-137.9632)))
                .splineTo(new Vector2d(7.5179, -61.8704), Math.toRadians(-90));

        TrajectoryActionBuilder mediumShoot4 = drive.actionBuilder(new Pose2d(7.5179, -61.8704, Math.toRadians(-90)))
                .setReversed(true)
                .splineTo(new Vector2d(11.035, -14.9189), Math.toRadians(-137.9632));



        // .strafeToLinearHeading(new Vector2d(5, 20), 90);

        waitForStart();
        while(opModeIsActive()) {


            Actions.runBlocking(
                    new SequentialAction(
                            farShootPreload.build(),
                            wallIntakeHPPre.build(),
                            wallIntakeHPPost.build(),
                            mediumShoot1.build(),
                            intakePreplace1.build(),
                            gateOpenBack.build(),
                            gateOpenForwards.build(),
                            mediumShoot2.build(),
                            intakePreplace2.build(),
                            mediumShoot3.build(),
                            intakePreplace3.build(),
                            mediumShoot4.build()
                    )
            );

            requestOpModeStop();

            robot.outtake.update();
        }

    }
}


