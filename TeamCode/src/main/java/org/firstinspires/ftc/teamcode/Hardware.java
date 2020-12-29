package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//test for notification
public class Hardware {

    public final double MINOR_MIN = 0, MINOR_MAX = 0.2;
    public final double SHOOTER_LIFT_MIN = 0, SHOOTER_LIFT_MAX = 0.5;
    public final double RING_PUSHER_MIN = 0, RING_PUSHER_MAX = 0.15;
    public final double RING_LIFT_MIN = 0, RING_LIFT_MAX = 0.6;

    Servo shooterLift = null; // 1 expansion hub
    Servo servoMinor = null; // 0 expansion hub
    Servo ringPusher = null; // 0 control hub
    Servo ringLift   = null; // 1 control hub

    DcMotor leftFront  = null; // 3 motor control hub
    DcMotor leftRear   = null; // 2 motor control hub
    DcMotor rightFront = null; // 1 motor control hub
    DcMotor rightRear  = null; // 0 motor control hub

    DcMotor intakeMajor = null; // 0 expansion hub
    DcMotor intakeMinor = null; // 1 expansion hub

    DcMotor shooter = null; // 2 expansion hub

    public void init(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        intakeMajor = hardwareMap.get(DcMotor.class, "intakeMajor");
        intakeMinor = hardwareMap.get(DcMotor.class, "intakeMinor");

        shooter = hardwareMap.get(DcMotor.class, "shooter");

        ringPusher = hardwareMap.get(Servo.class, "ringPusher");
        ringLift = hardwareMap.get(Servo.class, "ringLift");

        shooterLift = hardwareMap.get(Servo.class, "shooterLift");
        servoMinor = hardwareMap.get(Servo.class, "servoMinor");

        shooterLift.setPosition(SHOOTER_LIFT_MAX);
        servoMinor.setPosition(MINOR_MIN);
        ringPusher.setPosition(RING_PUSHER_MIN);
        ringLift.setPosition(RING_LIFT_MAX);

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMajor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void reset() {
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double move, double turn, double sideways) {
        double[] powers = new double[4];
        powers[0] = -move + turn + sideways;
        powers[1] = -move + turn - sideways;
        powers[2] = -move - turn - sideways;
        powers[3] = -move - turn + sideways;

        powers = normalize(powers);

        leftFront.setPower(-powers[0]);
        leftRear.setPower(-powers[1]);
        rightFront.setPower(-powers[2]);
        rightRear.setPower(-powers[3]);
    }

    private double[] normalize(double[] powers) {
        double max_value = Math.abs(powers[0]);
        for (double power : powers) {
            max_value = Math.max(max_value, power);
        }

        if (max_value > 1) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] /= max_value;
            }
        }
        return powers;
    }

    //0 - stop
    //1 - max power
    void shooterDo(boolean behaviour) {
        if (!behaviour) shooter.setPower(0);
        if (behaviour)  shooter.setPower(-1);
    }
}
