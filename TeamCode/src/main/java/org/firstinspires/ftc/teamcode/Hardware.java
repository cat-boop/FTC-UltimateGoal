package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.Map;

public class Hardware {

    public static Map<String, DcMotor> encoders = new HashMap<>();

    public final double CLAW_MIN = 0, CLAW_MAX = 1;
    public final double SHOOTER_ANGLE_MIN = 0.5, SHOOTER_ANGLE_MAX = 0.825;
    public final double RING_PUSHER_STOP = 0.5, RING_PUSHER_MOVE = 1;

    Servo ringPusher = null; // 0 control hub

    Servo servoClawLeft = null; // 0 expansion hub
    Servo servoClawRight = null; // 1 expansion hub

    Servo shooterAngle = null;

    DcMotor wobble = null;

    DcMotor ringLift = null; // 2 expansion hub

    DcMotor leftFront  = null; // 3 motor control hub
    DcMotor leftRear   = null; // 2 motor control hub
    DcMotor rightFront = null; // 1 motor control hub
    DcMotor rightRear  = null; // 0 motor control hub

    DcMotor intake = null;

    DcMotor shooter = null; // 2 expansion hub

    public Hardware() {
        //constructor without telemetry
    }

    public void init(HardwareMap hardwareMap) {

        //motor's
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        intake = hardwareMap.get(DcMotor.class, "intake");

        shooter = hardwareMap.get(DcMotor.class, "shooter");

        ringPusher = hardwareMap.get(Servo.class, "ringPusher");

        ringLift = hardwareMap.get(DcMotor.class, "ringLift");

        wobble = hardwareMap.get(DcMotor.class, "wobble");

        //servo's
        servoClawLeft = hardwareMap.get(Servo.class, "servoClawLeft");
        servoClawRight = hardwareMap.get(Servo.class, "servoClawRight");

        shooterAngle = hardwareMap.get(Servo.class, "shooterAngle");

        ringPusher.setPosition(RING_PUSHER_STOP);

        servoClawLeft.setPosition(CLAW_MIN);
        servoClawRight.setPosition(CLAW_MAX);

        shooterAngle.setPosition(SHOOTER_ANGLE_MAX);

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        ringLift.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        encoders.put("leftEncoder", shooter);
        encoders.put("rightEncoder", wobble);
        encoders.put("encoder", leftFront);
        encoders.put("liftEncoder", ringLift);
        //encoders.put("wobble", wobble);

        for (DcMotor dcMotor : encoders.values()) {
            dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }

    public void reset() {
        for (DcMotor dcMotor : encoders.values()) {
            dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void setPower(double move, double turn, double sideways) {
        //if (Math.abs(turn) >= 0.3 && Math.abs(turn) <= 0.9) turn /= 0.3;

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
            max_value = Math.max(max_value, Math.abs(power));
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
        if (behaviour)  shooter.setPower(-0.9);
    }

    void deployWobble() {
        servoClawLeft.setPosition(CLAW_MIN);
        servoClawRight.setPosition(CLAW_MAX);
    }

    void grabWobble() {
        servoClawLeft.setPosition(CLAW_MAX);
        servoClawRight.setPosition(CLAW_MIN);
    }
}
