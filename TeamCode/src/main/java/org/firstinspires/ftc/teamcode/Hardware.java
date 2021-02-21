package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;

public class Hardware {

    public static Map<String, DcMotor> encoders = new HashMap<>();

    public final double CLAW_MIN = 0, CLAW_MAX = 1;
    public final double TOWER_ANGLE_MIN = 0.5, TOWER_ANGLE_MAX = 1;
    public final double RING_PUSHER_STOP = 0.5, RING_PUSHER_MOVE = 1;

    Servo ringPusherLeft = null;  // 1 control hub
    Servo ringPusherRight = null; // 2 control hub

    Servo servoClawLeft = null; // 0 expansion hub
    Servo servoClawRight = null; // 1 expansion hub

    Servo towerAngle = null;

    DcMotor wobble = null;

    DcMotor ringLift = null; // 2 expansion hub

    DcMotor leftFront  = null; // 3 motor control hub
    DcMotor leftRear   = null; // 2 motor control hub
    DcMotor rightFront = null; // 1 motor control hub
    DcMotor rightRear  = null; // 0 motor control hub

    DcMotor intake = null;

    DcMotorEx shooter = null; // 2 expansion hub

    TouchSensor isLiftDown, isLiftUp, ringsIsNone;
    public static boolean needLiftDown = false, needLiftUp = false, needStartShoot = false;

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

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        ringLift = hardwareMap.get(DcMotor.class, "ringLift");

        wobble = hardwareMap.get(DcMotor.class, "wobble");

        //servo's
        ringPusherLeft = hardwareMap.get(Servo.class, "ringPusherLeft");
        ringPusherRight = hardwareMap.get(Servo.class, "ringPusherRight");

        servoClawLeft = hardwareMap.get(Servo.class, "servoClawLeft");
        servoClawRight = hardwareMap.get(Servo.class, "servoClawRight");

        towerAngle = hardwareMap.get(Servo.class, "shooterAngle");

        //button's
        isLiftDown = hardwareMap.get(TouchSensor.class, "isLiftDown");
        isLiftUp = hardwareMap.get(TouchSensor.class, "isLiftUp");
        ringsIsNone = hardwareMap.get(TouchSensor.class, "ringsIsNone");

        //servo start position's
        ringPusherLeft.setPosition(RING_PUSHER_STOP);
        ringPusherRight.setPosition(RING_PUSHER_STOP);

        servoClawLeft.setPosition(CLAW_MIN);
        servoClawRight.setPosition(CLAW_MAX);

        towerAngle.setPosition(TOWER_ANGLE_MAX);

        //motor mode's
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        ringLift.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //encoders initialization
        encoders.put("leftEncoder", leftRear);
        encoders.put("rightEncoder", wobble);
        encoders.put("encoder", rightRear);
        encoders.put("wobble", ringLift);

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

        double maxValue = Math.abs(powers[0]);
        for (double power : powers) {
            maxValue = Math.max(maxValue, Math.abs(power));
        }

        if (maxValue > 1) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] /= maxValue;
            }
        }

        leftFront.setPower(-powers[0]);
        leftRear.setPower(-powers[1]);
        rightFront.setPower(-powers[2]);
        rightRear.setPower(-powers[3]);
    }

    public enum Claw {
        OPEN,
        CLOSE
    }

    void clawCommand(Claw control) {
        if (control == Claw.OPEN) {
            servoClawLeft.setPosition(CLAW_MIN);
            servoClawRight.setPosition(CLAW_MAX);
        }
        if (control == Claw.CLOSE) {
            servoClawLeft.setPosition(CLAW_MAX);
            servoClawRight.setPosition(CLAW_MIN);
        }
    }

    public enum TowerState {
        STOP,
        SHOOTER_ON,
        PUSHER_ON
    }

    void shooterCommand(TowerState towerState) {
        if (towerState == TowerState.STOP) shooter.setVelocity(0);
        if (towerState == TowerState.SHOOTER_ON)  shooter.setVelocity(6000);
    }

    void pusherCommand(TowerState towerState) {
        if (towerState == TowerState.STOP) {
            ringPusherLeft.setPosition(RING_PUSHER_STOP);
            ringPusherRight.setPosition(RING_PUSHER_STOP);
        }
        if (towerState == TowerState.PUSHER_ON) {
            ringPusherLeft.setPosition(-RING_PUSHER_MOVE);
            ringPusherRight.setPosition(RING_PUSHER_MOVE);
        }
    }

    public void putLiftDown() {
        if (isLiftDown.isPressed()) ringLift.setPower(-0.6);
        else {
            ringLift.setPower(0);
            needLiftDown = false;
        }
    }

    public void putLiftUp(double speed) {
        if (isLiftUp.isPressed() && ringsIsNone.isPressed()) ringLift.setPower(speed);
        else if (!isLiftUp.isPressed() || !ringsIsNone.isPressed()){
            ringLift.setPower(0);
            needLiftUp = false;
        }
    }

    public void shoot() {
        shooterCommand(TowerState.SHOOTER_ON);
        if (!ringsIsNone.isPressed()) {
            needStartShoot = false;
            shooterCommand(TowerState.STOP);
        }

        if (!needLiftUp) {
            pusherCommand(TowerState.PUSHER_ON);
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            while (timer.milliseconds() < 1000);
            pusherCommand(TowerState.STOP);
            needLiftUp = true;
        }
    }
}
