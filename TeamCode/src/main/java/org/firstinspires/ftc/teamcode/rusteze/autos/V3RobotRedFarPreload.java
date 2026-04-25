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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.rusteze.RobotConstants;
import org.firstinspires.ftc.teamcode.rusteze.subsystems.Intake;
import org.firstinspires.ftc.teamcode.rusteze.subsystems.RobotV2;
import org.firstinspires.ftc.teamcode.rusteze.utility.RegressionContainer;

@Autonomous(name="V3 Red Far", group="main")
public class V3RobotRedFarPreload extends LinearOpMode {
    boolean outtakeEnabled = false;

    public void runOpMode() {

        RobotConstants.teamColor = RobotConstants.Color.RED;

        RobotV2 robot = new RobotV2(hardwareMap);
        ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime totalTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        class SpinUpFlywheel implements Action {
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    robot.outtake.useControlSystem = true;
                    robot.outtake.setTarget(3450);
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

        class OuttakeOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                packet.put("Outtake Enabled", true);
                outtakeEnabled = true;
                return false;
            }
        }

        class OuttakeOff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                packet.put("Outtake Enabled", false);
                outtakeEnabled = false;
                return false;
            }
        }


        class FlywheelUpdating implements Action {
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    totalTime.reset();
                    initialized = true;
                    robot.outtake.useControlSystem = true;
                }

                packet.put("Flywheel Updating", true);

                if (totalTime.time() < 30000) {
                    if (outtakeEnabled) {
                        robot.outtake.setTarget(6000);
                        robot.outtake.setHood(0.2);
                    } else {
                        robot.outtake.setTarget(0);
                    }
                    robot.update();
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
                    robot.intake.autoFarTransfer();
                    initialized = true;
                    elapsedTime.reset();
                }

                packet.put("Elapsed Time", elapsedTime.time());
                if (elapsedTime.time() < 2500) {
                    return true;
                } else {
                    outtakeEnabled = false;
                    robot.intake.disable();
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

        class TransferOn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    robot.setState(RobotV2.State.INTAKE);
                    robot.intake.enableTransfer();
                    initialized = true;
                    elapsedTime.reset();
                }

                packet.put("Elapsed Time", elapsedTime.time());

                return false;
            }
        }

        class SetTurret implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                packet.put("Turret Position", 295);
                robot.turret.setSmartTargetPosition(295);
                return false;
            }
        }

        class FinalTurret implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                packet.put("Turret Position", 235);
                robot.turret.setSmartTargetPosition(235);
                return false;
            }
        }

        class ResetTurret implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                packet.put("Turret Position", 0);
                robot.turret.setTargetPosition(0);
                return false;
            }
        }


        Pose2d initialPose = new Pose2d(59.3, 18.4, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder shootPreload = drive.actionBuilder(initialPose)
                .afterTime(0.0, new FlywheelUpdating())
                .afterTime(0.1, new OuttakeOn())
                .afterTime(0.1, new SetTurret())
                .strafeToLinearHeading(new Vector2d(45, 15), Math.toRadians(90))

                .waitSeconds(1.5)

                // Preloads
                .afterTime(0.0, new Shoot())

                .waitSeconds(3.0)

                // Intake from HP
                .afterTime(0.0, new IntakeOn())
                .strafeToLinearHeading(new Vector2d(59.3, 18.4), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(59.3, 64), Math.toRadians(90))
                .afterTime(0.1, new IntakeOff())
                .afterTime(0.2, new OuttakeOn())
                .strafeToLinearHeading(new Vector2d(45, 15), Math.toRadians(90))
                .afterTime(0.0, new Shoot())
                .waitSeconds(3.0)

                // Intake from HP
                .afterTime(0.0, new IntakeOn())
                .strafeToLinearHeading(new Vector2d(59.3, 18.4), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(59.3, 64), Math.toRadians(90))
                .afterTime(0.1, new IntakeOff())
                .afterTime(0.2, new OuttakeOn())
                .strafeToLinearHeading(new Vector2d(45, 15), Math.toRadians(90))
                .afterTime(0.0, new Shoot())
                .waitSeconds(3.0)

                // Intake from HP
                .afterTime(0.0, new IntakeOn())
                .strafeToLinearHeading(new Vector2d(59.3, 18.4), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(59.3, 64), Math.toRadians(90))
                .afterTime(0.1, new IntakeOff())
                .afterTime(0.2, new OuttakeOn())
                .strafeToLinearHeading(new Vector2d(45, 15), Math.toRadians(90))
                .afterTime(0.0, new Shoot())
                .waitSeconds(3.0)


                .afterTime(0, new ResetTurret())
                .strafeToLinearHeading(new Vector2d(20, 15), Math.toRadians(90))

                .waitSeconds(10.0)


                ;

                /*
                // Gate #2
                .strafeToLinearHeading(new Vector2d(7.0, 68), Math.toRadians(90))
                .waitSeconds(0.8)

                // Intake from Gate
                .afterTime(0, new IntakeOn())
                .strafeToLinearHeading(new Vector2d(20, 72), Math.toRadians(170))
                .waitSeconds(3.0)

                // Shoot from Gate
                .afterTime(0.0, new OuttakeOn())
                .strafeToLinearHeading(new Vector2d(-6, 20), Math.toRadians(90))
                .afterTime(0.2, new Shoot())
                 */



        waitForStart();
        while(opModeIsActive()) {

            robot.turret.initializeTurret();

            Actions.runBlocking(
                    new SequentialAction(
                            shootPreload.build()
                    )
            );

            requestOpModeStop();
        }

    }
}


