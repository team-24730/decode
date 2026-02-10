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

@Disabled
@Autonomous(name="Red Close M1GM2M3FHF", group="main")
public class RedCloseM1GM2M3FHF extends LinearOpMode {

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
                    robot.setOuttakeTarget(3900);
                    robot.outtake.setHood(0.35);
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
                    robot.setOuttakeTarget(4700);
                    robot.outtake.setHood(0.25);
                    robot.outtake.update();
                    initialized = true;
                    elapsedTime.reset();
                }
                packet.put("Elapsed Time", elapsedTime.time());

                if (elapsedTime.time() < 2000) {
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
                    robot.transfer.setPower(0.65);
                    initialized = true;
                    elapsedTime.reset();
                }

                packet.put("Elapsed Time", elapsedTime.time());
                if (elapsedTime.time() < 1250) {
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
                robot.transfer.setPower(0.7);
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


        Pose2d initialPose = new Pose2d(-56, 34, Math.toRadians(127.8));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder shootPreload = drive.actionBuilder(initialPose)
                /* Preloads */
                .afterTime(0, new SpinUpFlywheelMedium())
                .afterTime(1.4, new ShootClose())
                .strafeToLinearHeading(new Vector2d(-18, 4), Math.toRadians(135)) // shoot preload
                .waitSeconds(1.2) // see if time can be saved

                /* First Spike */
                .afterTime(0.6, new IntakeOn())
                .splineTo(new Vector2d(-13, 50), Math.toRadians(90))
                .afterTime(0.0, new IntakeOff())
                .waitSeconds(0.2)
                .afterTime(0.71, new SpinUpFlywheelMedium())
                .afterTime(1.3, new ShootClose())
                .strafeToLinearHeading(new Vector2d(-22, 4), Math.toRadians(135))
                .waitSeconds(0.6)

                /* Second Spike */
                .strafeToLinearHeading(new Vector2d(6, 6), Math.toRadians(90))
                .afterTime(0.0, new IntakeOn())
                .strafeToLinearHeading(new Vector2d(6, 44), Math.toRadians(90))
                .afterTime(0.5, new IntakeOff())
                .strafeToLinearHeading(new Vector2d(-2, 60), Math.toRadians(0)) // open gate
                .afterTime(0.9, new SpinUpFlywheelMedium())
                .afterTime(1.9, new ShootClose())
                .strafeToLinearHeading(new Vector2d(-20, 4), Math.toRadians(140))
                .waitSeconds(0.75)

                /* Third Spike */
                .strafeToLinearHeading(new Vector2d(30, 12), Math.toRadians(90))
                .afterTime(0, new IntakeOn())
                .strafeToLinearHeading(new Vector2d(30, 44), Math.toRadians(90))
                .afterTime(0, new IntakeOff())
                .afterTime(0.7, new SpinUpFlywheelMedium())
                .afterTime(1.5, new ShootClose())
                .strafeToLinearHeading(new Vector2d(-22, 6), Math.toRadians(137))
                .waitSeconds(0.6)

                /* HP Zone */
                .strafeToLinearHeading(new Vector2d(30, 56), Math.toRadians(30))
                .afterTime(0, new IntakeOn())
                .strafeToLinearHeading(new Vector2d(72, 74), Math.toRadians(0))
                .afterTime(0, new IntakeOff()).
                afterTime(0.6, new SpinUpFlywheelMedium())
                .afterTime(2.7, new ShootClose())
                .strafeToLinearHeading(new Vector2d(-21, 5), Math.toRadians(137))
                .waitSeconds(0.75)



                /* Park */
                .strafeToLinearHeading(new Vector2d(-5, 16), Math.toRadians(90))

                ;



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


