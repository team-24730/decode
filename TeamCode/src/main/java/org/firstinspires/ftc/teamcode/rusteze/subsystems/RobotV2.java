package org.firstinspires.ftc.teamcode.rusteze.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotV2 {
    public Drivetrain drivetrain;
    public Intake intake;
    public Outtake outtake;
    public Lift lift;
    public Turret turret;

    public enum State {
        IDLE,
        INTAKE,
        SHOOTING,
        LIFT,
        RELOCALIZING
    }
    State robotState = State.IDLE;

    public RobotV2(HardwareMap hwMap) {
        drivetrain = new Drivetrain(hwMap);
        intake = new Intake(hwMap);
        outtake = new Outtake(hwMap);
        lift = new Lift(hwMap);
        turret = new Turret(hwMap);
    }

    public State getState() {
        return robotState;
    }

    public void setState(State state) {
        this.robotState = state;
    }

    public void update() {
        intake.update();
        outtake.update();
        turret.update();
        drivetrain.update();
    }
}
