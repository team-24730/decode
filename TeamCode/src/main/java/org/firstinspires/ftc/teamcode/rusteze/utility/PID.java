package org.firstinspires.ftc.teamcode.rusteze.utility;

public class PID {
    double p, i, d;
    long lastTime;
    double lastError;
    double integral;

    public PID(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
        lastError = 0;
        lastTime = System.nanoTime();
        integral = 0;
    }

    public double calculate(double error) {
        long currentTime = System.nanoTime();
        double loopTime = (currentTime - lastTime) / 1.0e9; // time length of the current loop (divided by 1,000,000,000 to convert to seconds)
        lastTime = currentTime;

        integral += error * loopTime; // height of the rectangle is the error and the width is the length of the current loop (loopTime)

        double p_power = p * error;
        double i_power = i * integral;
        double d_power = ((error - lastError) / loopTime) * d; // calculate slope using rise (change in error) over run (change in time)

        lastError = error;

        return p_power + i_power + d_power;
    }

    public void resetIntegral() {
        integral = 0;
    }
}
