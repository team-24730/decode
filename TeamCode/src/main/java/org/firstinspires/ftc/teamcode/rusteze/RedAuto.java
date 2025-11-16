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

@Autonomous(name="Red Auto")
public class RedAuto extends LinearOpMode {


    public void runOpMode() {

        Outtake outtake = new Outtake(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Robot robot = new Robot(hardwareMap);
        ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        class OuttakeClose implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    robot.setOuttakeTarget(3000);
                    outtake.setHood(0.97);
                    initialized = true;
                    elapsedTime.reset();
                }

                packet.put("Elapsed Time", elapsedTime);
                if (elapsedTime.time() < 3000) {
                    if (elapsedTime.time() > 1000) {
                        transfer.setPower(1);
                    }
                    outtake.update();
                    return true;
                } else {
                    robot.setOuttakeTarget(0);
                    transfer.setPower(0);
                    return false;
                }
            }
        }


        Pose2d initialPose = new Pose2d(-50, 48, Math.toRadians(216));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-40, 39));

        TrajectoryActionBuilder closeOut = drive.actionBuilder(new Pose2d(-40, 39, Math.toRadians(216)))
                .waitSeconds(10)
                .lineToXSplineHeading(0, Math.toRadians(0));

        waitForStart();
        while(opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            tab1.build(),
                            new OuttakeClose(),
                            closeOut.build()
                    )
            );

            outtake.update();
        }

    }
}


