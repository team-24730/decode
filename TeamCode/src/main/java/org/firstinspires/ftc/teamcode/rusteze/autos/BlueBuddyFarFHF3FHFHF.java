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
@Autonomous(name="Blue Buddy Far FHF3FHFHF", group="main")
public class BlueBuddyFarFHF3FHFHF extends LinearOpMode {

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
                robot.transfer.setPower(0.3);
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


        Pose2d initialPose = new Pose2d(60, -17, Math.toRadians(-180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder shootPreload = drive.actionBuilder(initialPose)
                /* Preloads */
                .afterTime(0, new SpinUpFlywheelFar())
                .afterTime(0.8, new ShootFar())
                .strafeToLinearHeading(new Vector2d(55, -12), Math.toRadians(-155)) // shoot preload
                .waitSeconds(1.0) // see if time can be saved

                /* Initial HP Zone */
                .afterTime(0.4, new IntakeOn())
                .strafeToLinearHeading(new Vector2d(50, -70), Math.toRadians(-70)) // intake hp
                .strafeToLinearHeading(new Vector2d(62, -80), Math.toRadians(-0)) // shift
                .strafeToLinearHeading(new Vector2d(65, -80), Math.toRadians(-0)) // shift
                .afterTime(0, new IntakeOff())
                .afterTime(1.1, new SpinUpFlywheelFar())
                .afterTime(2.1, new ShootFar())
                .strafeToLinearHeading(new Vector2d(55, -12), Math.toRadians(-160)) // shoot hp
                .waitSeconds(1.0) // see if time can be saved

                /* Third Spike Mark */
                .afterTime(0.3, new IntakeOn())
                .splineTo(new Vector2d(30, -50), Math.toRadians(-90)) // intake third spike
                .afterTime(0, new IntakeOff())
                .afterTime(0.6, new SpinUpFlywheelFar())
                .afterTime(1.2, new ShootFar()) // see if time can be saved
                .strafeToLinearHeading(new Vector2d(55, -12), Math.toRadians(-160)) // shoot third spike
                .waitSeconds(1.0)

                /* Second HP Intake */
                .afterTime(0.4, new IntakeOn())
                .strafeToLinearHeading(new Vector2d(50, -70), Math.toRadians(-70)) // intake hp
                .strafeToLinearHeading(new Vector2d(62, -80), Math.toRadians(-0)) // shift
                .strafeToLinearHeading(new Vector2d(65, -80), Math.toRadians(-0)) // shift
                .afterTime(0, new IntakeOff())
                .afterTime(1.1, new SpinUpFlywheelFar())
                .afterTime(1.9, new ShootFar())
                .strafeToLinearHeading(new Vector2d(55, -12), Math.toRadians(-160)) // shoot hp
                .waitSeconds(1.0) // see if time can be saved

                /* Third HP Intake */
                .afterTime(0.4, new IntakeOn())
                .strafeToLinearHeading(new Vector2d(30, -70), Math.toRadians(-70)) // intake hp
                .strafeToLinearHeading(new Vector2d(62, -80), Math.toRadians(-0)) // shift
                .strafeToLinearHeading(new Vector2d(65, -80), Math.toRadians(-0)) // shift
                .afterTime(0, new IntakeOff())
                .afterTime(1.1, new SpinUpFlywheelFar())
                .afterTime(1.9, new ShootFar())
                .strafeToLinearHeading(new Vector2d(55, -12), Math.toRadians(-160)) // shoot hp
                .waitSeconds(0.8)
                .strafeToLinearHeading(new Vector2d(40, -12), Math.toRadians(-160)) // park

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


