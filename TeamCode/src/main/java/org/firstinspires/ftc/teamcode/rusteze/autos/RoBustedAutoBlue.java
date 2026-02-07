package org.firstinspires.ftc.teamcode.rusteze.autos;

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
import org.firstinspires.ftc.teamcode.rusteze.Robot;
import org.firstinspires.ftc.teamcode.rusteze.RobotConstants;

@Disabled
@Autonomous(name="RoBusted Auto Blue", group="not main")
public class RoBustedAutoBlue extends LinearOpMode {



    public void runOpMode() {

        RobotConstants.teamColor = RobotConstants.Color.BLUE;

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
                    robot.setOuttakeTarget(4650);
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


        Pose2d initialPose = new Pose2d(55, -7, Math.toRadians(-180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder shootPreload = drive.actionBuilder(initialPose)
                .afterTime(0, new SpinUpFlywheelFar())
                .afterTime(1.55, new ShootFar())
                .strafeToLinearHeading(new Vector2d(52, -12), Math.toRadians(-158)) // initial shooting pose
                .waitSeconds(1.9)
                .strafeToLinearHeading(new Vector2d(28, -10), Math.toRadians(-90)) // prepare to intake third spike mark
                .afterTime(0, new IntakeOn())
                .strafeToLinearHeading(new Vector2d(32, -46), Math.toRadians(-90)) // intake third spike mark
                .afterTime(0, new IntakeOff())
                .afterTime(0.55, new SpinUpFlywheelFar())
                .afterTime(1.2, new ShootFar())
                .strafeToLinearHeading(new Vector2d(52, -12), Math.toRadians(-161)) // shoot third spike mark
                .waitSeconds(1.7)
                .afterTime(0.5, new IntakeOn())
                .strafeToLinearHeading(new Vector2d(58, -49), Math.toRadians(-90)) // intake hp
                .strafeToLinearHeading(new Vector2d(64, -46), Math.toRadians(-70)) // intake hp shift
                .afterTime(0, new IntakeOff())
                .afterTime(0.6, new SpinUpFlywheelFar())
                .strafeToLinearHeading(new Vector2d(52, -12), Math.toRadians(-163)) // shoot hp zone
                .afterTime(0, new ShootFar())
                .waitSeconds(2.5)

                .afterTime(0, new IntakeOn())
                .strafeToLinearHeading(new Vector2d(62, -48), Math.toRadians(-90)) // intake hp
                .strafeToLinearHeading(new Vector2d(66, -46), Math.toRadians(-70)) // intake hp shift
                .afterTime(0, new IntakeOff())
                .afterTime(0.6, new SpinUpFlywheelFar())
                .strafeToLinearHeading(new Vector2d(52, -12), Math.toRadians(-163)) // shoot hp zone
                .afterTime(0.2, new ShootFar())
                .waitSeconds(2.1)

                .afterTime(0, new IntakeOn())
                .strafeToLinearHeading(new Vector2d(62, -48), Math.toRadians(-90)) // intake hp
                .strafeToLinearHeading(new Vector2d(66, -46), Math.toRadians(-70)) // intake hp shift
                .afterTime(0, new IntakeOff())
                .afterTime(0.6, new SpinUpFlywheelFar())
                .strafeToLinearHeading(new Vector2d(52, -12), Math.toRadians(-163)) // shoot hp zone
                .afterTime(0.2, new ShootFar())
                .waitSeconds(2.1)

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


