package org.firstinspires.ftc.teamcode.rusteze.autos;

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
import org.firstinspires.ftc.teamcode.rusteze.RobotConstants;
import org.firstinspires.ftc.teamcode.rusteze.subsystems.RobotV2;

@Autonomous(name="Red Close Main", group="main")
public class V2RobotRedClose extends LinearOpMode {

    public void runOpMode() {

        RobotConstants.teamColor = RobotConstants.Color.RED;

        RobotV2 robot = new RobotV2(hardwareMap);
        ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        class SpinUpFlywheelClose implements Action {
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    robot.outtake.useControlSystem = true;
                    robot.outtake.setTarget(3350);
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
                    robot.outtake.useControlSystem = true;
                    robot.outtake.setTarget(3300);
                    robot.outtake.setHood(0.6);
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
                    robot.outtake.useControlSystem = true;
                    robot.outtake.setTarget(4700);
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

        class Shoot implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    robot.intake.enableTransfer();
                    initialized = true;
                    elapsedTime.reset();
                }

                packet.put("Elapsed Time", elapsedTime.time());
                if (elapsedTime.time() < 2000) {
                    robot.update(); // update the robot so the PID continues working and transfer works
                    if (elapsedTime.time() > 1500) { robot.outtake.setTarget(0); }
                    return true;
                } else {
                    robot.outtake.useControlSystem = false;
                    robot.intake.disable();
                    robot.outtake.setTarget(0);
                    robot.outtake.setPower(0);
                    robot.update();
                    return false;
                }
            }
        }

        class IntakeOn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    robot.setState(RobotV2.State.INTAKE);
                    robot.intake.enable();
                    initialized = true;
                    elapsedTime.reset();
                }

                packet.put("Elapsed Time", elapsedTime.time());
                if (elapsedTime.time() < 500) {
                    robot.update(); // update the robot so the intake logic works
                    return true;
                } else {
                    return false;
                }
            }
        }

        class IntakeOff implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    robot.intake.disable();
                    initialized = true;
                    elapsedTime.reset();
                }

                packet.put("Elapsed Time", elapsedTime.time());
                if (elapsedTime.time() < 500) {
                    robot.update(); // update the robot so the intake logic works
                    return true;
                } else {
                    return false;
                }
            }
        }


        Pose2d initialPose = new Pose2d(-56, 34, Math.toRadians(127.8));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder shootPreload = drive.actionBuilder(initialPose)
                /* Preloads */
                .afterTime(0, new SpinUpFlywheelMedium())
                .afterTime(1.4, new Shoot())
                .strafeToLinearHeading(new Vector2d(-24, 4), Math.toRadians(135)) // shoot preload
                .waitSeconds(2.0) // see if time can be saved

                /* First Spike */
                .afterTime(0.1, new IntakeOn())
                .splineTo(new Vector2d(-13, 50), Math.toRadians(90), new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 40;
                    }
                })
                .afterTime(0.5, new IntakeOff())
                .waitSeconds(0.4)
                .afterTime(0.71, new SpinUpFlywheelMedium())
                .afterTime(1.5, new Shoot())
                .strafeToLinearHeading(new Vector2d(-22, 4), Math.toRadians(138))
                .waitSeconds(1.5)

                /* Second Spike */
                .strafeToLinearHeading(new Vector2d(6, 6), Math.toRadians(90))
                .afterTime(0.0, new IntakeOn())
                .strafeToLinearHeading(new Vector2d(12, 60), Math.toRadians(90), new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 40;
                    }
                })
                .afterTime(1.0, new IntakeOff())
                .strafeToLinearHeading(new Vector2d(-2, 60), Math.toRadians(0)) // open gate
                .waitSeconds(0.5)
                .afterTime(0.9, new SpinUpFlywheelMedium())
                .afterTime(1.9, new Shoot())
                .strafeToLinearHeading(new Vector2d(-24, 4), Math.toRadians(137))
                .waitSeconds(2.0)

                /* Third Spike */
                .strafeToLinearHeading(new Vector2d(30, 12), Math.toRadians(90))
                .afterTime(0, new IntakeOn())
                .strafeToLinearHeading(new Vector2d(30, 60), Math.toRadians(90), new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 30;
                    }
                })
                .afterTime(1.1, new IntakeOff())
                .afterTime(0.7, new SpinUpFlywheelMedium())
                .afterTime(1.5, new Shoot())
                .strafeToLinearHeading(new Vector2d(-26, 8), Math.toRadians(135))
                .waitSeconds(2.0)

                /* Park */
                .strafeToLinearHeading(new Vector2d(-5, 16), Math.toRadians(90))

                ;



        waitForStart();
        while(opModeIsActive()) {

            robot.turret.initializeTurret();
            robot.turret.setTargetPosition(0);

            Actions.runBlocking(
                    new SequentialAction(
                            shootPreload.build()
                    )
            );

            requestOpModeStop();
        }

    }
}


