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

@Autonomous(name="Blue Auto")
public class BlueAuto extends LinearOpMode {


    public void runOpMode() {

        Robot robot = new Robot(hardwareMap);
        ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        class OuttakeClose implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    robot.setOuttakeTarget(3000);
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


        Pose2d initialPose = new Pose2d(-56, -34, Math.toRadians(-127.8));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder closeShootPreload = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-46, -22));

        TrajectoryActionBuilder firstIntakeBack = drive.actionBuilder(new Pose2d(-46, -22, Math.toRadians(-127.8)))
                .strafeToLinearHeading(new Vector2d(-18, 0), Math.toRadians(-90));

        TrajectoryActionBuilder firstIntakeForward = drive.actionBuilder(new Pose2d(-18, 0, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(-18, -25), Math.toRadians(-90));

        TrajectoryActionBuilder closeShootFirst = drive.actionBuilder(new Pose2d(-18, -25, Math.toRadians(-90)))
                .afterTime(0.2, new OuttakeBackwards())
                .strafeToLinearHeading(new Vector2d(-46, -22), Math.toRadians(-127.8));

        TrajectoryActionBuilder secondIntakeBack = drive.actionBuilder(new Pose2d(-46, -22, Math.toRadians(-127.8)))
                .strafeToLinearHeading(new Vector2d(8, 0), Math.toRadians(-90));

        TrajectoryActionBuilder secondIntakeForward = drive.actionBuilder(new Pose2d(8, 0, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(8, -25), Math.toRadians(-90));

        TrajectoryActionBuilder closeShootSecond = drive.actionBuilder(new Pose2d(8, -25, Math.toRadians(-90)))
                .afterTime(0.2, new OuttakeBackwards())
                .strafeToLinearHeading(new Vector2d(-46, -22), Math.toRadians(-127.8));

        TrajectoryActionBuilder thirdIntakeBack = drive.actionBuilder(new Pose2d(-46, -22, Math.toRadians(-127.8)))
                .strafeToLinearHeading(new Vector2d(32, 0), Math.toRadians(-90));

        TrajectoryActionBuilder thirdIntakeForward = drive.actionBuilder(new Pose2d(32, 0, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(32, -25), Math.toRadians(-90));

        TrajectoryActionBuilder thirdShootSecond = drive.actionBuilder(new Pose2d(32, -25, Math.toRadians(-90)))
                .afterTime(0.2, new OuttakeBackwards())
                .strafeToLinearHeading(new Vector2d(-46, -22), Math.toRadians(-127.8));

        TrajectoryActionBuilder finalPosition = drive.actionBuilder(new Pose2d(-46, -22, Math.toRadians(-127.8)))
                .strafeToLinearHeading(new Vector2d(0, -20), Math.toRadians(0));



        // .strafeToLinearHeading(new Vector2d(5, 20), 90);

        waitForStart();
        while(opModeIsActive()) {


            Actions.runBlocking(
                    new SequentialAction(
                            closeShootPreload.build(),
                            new OuttakeClose(),
                            firstIntakeBack.build(),
                            new IntakeOn(),
                            firstIntakeForward.build(),
                            new IntakeOff(),
                            closeShootFirst.build(),
                            new OuttakeClose(),
                            secondIntakeBack.build(),
                            new IntakeOn(),
                            secondIntakeForward.build(),
                            new IntakeOff(),
                            closeShootSecond.build(),
                            new OuttakeClose(),
                            thirdIntakeBack.build(),
                            new IntakeOn(),
                            thirdIntakeForward.build(),
                            new IntakeOff(),
                            thirdShootSecond.build(),
                            new OuttakeClose(),
                            finalPosition.build()
                    )
            );

            requestOpModeStop();

            robot.outtake.update();
        }

    }
}


