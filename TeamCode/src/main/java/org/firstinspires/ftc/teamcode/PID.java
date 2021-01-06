package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class PID {
    double kP, kI, kD;

    double error = 0, past_error = Double.NaN;

    public PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double apply(double error) {
        // past_error = error;
        return clipValue(iRegulator(error));
    }

    private double iRegulator(double error) { return error * kP; }

    private double clipValue(double value) {
        double min_negative_value = -1, max_negative_value = -0.2;
        double max_positive_value = 1, min_positive_value = 0.2;
        return (value > 0) ? Range.clip(value, min_positive_value, max_positive_value) : Range.clip(value, min_negative_value, max_negative_value);
    }
}
