package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Gyroscope {

    private BNO055IMU imu;
    private PID pid = new PID(0.01, 0, 0);

    private final int MOD = 360;

    public void init(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard

    }

    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public double turnTo(double currentAngle, double targetAngle) {
        double error = format(targetAngle, currentAngle);
        return pid.apply(error);
    }

    public double format(double valueOne, double valueTwo) {
        return clamp180(to360(valueOne - clamp360(valueTwo)));
    }

    private double clamp360(double angle) {
        if (angle < 0) return angle + 360;
        return angle;
    }

    // > 0 and < 360
    private double to360(double angle) {
        return (angle + MOD) % MOD;
    }

    //[0;360] -> [-180;180]
    private double clamp180(double angle) {
        if (angle > 180) return angle - 360;
        return angle;
    }
}
