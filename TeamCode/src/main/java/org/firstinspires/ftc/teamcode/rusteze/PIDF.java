package org.firstinspires.ftc.teamcode.rusteze;

public class PIDF extends PID {
    double f;
    double threshold;

    public PIDF(double p, double i, double d, double f, double threshold) {
        super(p, i, d);
        this.f = f;
        this.threshold = threshold;
    }

    @Override
    public double calculate(double error) {
        double power = super.calculate(error);

        if (Math.abs(error) > threshold) {
            power += Math.signum(error) * f;
        } else {
            super.resetIntegral();
        }

        return power;
    }

}
