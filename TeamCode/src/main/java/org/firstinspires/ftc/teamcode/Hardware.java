package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.PIDFValues.kP;
import static org.firstinspires.ftc.teamcode.PIDFValues.kI;
import static org.firstinspires.ftc.teamcode.PIDFValues.kD;
import static org.firstinspires.ftc.teamcode.PIDFValues.kF;

public class Hardware {

    public static Map<String, DcMotor> encoders = new HashMap<>();

    private final double TOWER_ANGLE_MAX = 0.4, TOWER_ANGLE_MIN = 1, POSISHION_TO_SHOOT = 0.8;//need to be corrected

    //private final double RING_PUSHER_STOP = 0.5, RING_PUSHER_MOVE = 1;
    //private final double LIFT_DOOR_LEFT_OPEN = 0.6, LIFT_DOOR_LEFT_CLOSE = 0.46;
    //private final double LIFT_DOOR_RIGHT_OPEN = 0.65, LIFT_DOOR_RIGHT_CLOSE = 0.75;
    private final double RING_PUSHER_ON = 0.69 ,RING_PUSHER_BACK = 0.39;

    public double getMinTowerAngle() { return TOWER_ANGLE_MIN; }
    public double getMaxTowerAngle() { return TOWER_ANGLE_MAX; }

    public boolean toshoot = true;

    public Servo manipulatorReturner = null; // 1 Cont

    public Servo ringPusher = null;// 4 ex

    //public Servo liftDoorLeft = null;
    //public Servo liftDoorRight = null;

    //public Servo ringPusherLeft = null;  // 1 control hub
    //public Servo ringPusherRight = null; // 2 control hub

    public Servo servoClawLeft = null; // 3 Ex hub
    public Servo servoClawRight = null; // 4 ex hub

    public Servo towerAngle = null; //5 Ex hub

    public DcMotor manipulator = null;

    //public DcMotor ringLift = null; // 2 expansion hub

    public DcMotor leftFront  = null;
    public DcMotor leftRear   = null;
    public DcMotor rightFront = null;
    public DcMotor rightRear  = null;

    public DcMotor intake = null;

    public DcMotorEx shooter = null; // 2 expansion hub

    ElapsedTime pusherTimer = new ElapsedTime();

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

        //ringLift = hardwareMap.get(DcMotor.class, "ringLift");

        manipulator = hardwareMap.get(DcMotor.class, "manipulator");

        //servo's
        //manipulatorReturner = hardwareMap.get(Servo.class, "manipulatorReturner");

        //liftDoorLeft = hardwareMap.get(Servo.class, "liftDoorLeft");
        //liftDoorRight = hardwareMap.get(Servo.class, "liftDoorRight");

        //ringPusherLeft = hardwareMap.get(Servo.class, "ringPusherLeft");
        //ringPusherRight = hardwareMap.get(Servo.class, "ringPusherRight");

        ringPusher = hardwareMap.get(Servo.class, "ringPusher");

        servoClawLeft = hardwareMap.get(Servo.class, "servoClawLeft");
        servoClawRight = hardwareMap.get(Servo.class, "servoClawRight");

        towerAngle = hardwareMap.get(Servo.class, "shooterAngle");

        //button's

        //servo start position's
        manipulatorCommand(ManipulatorState.DISASSEMBLED);

        //doorCommand(Door.OPEN);

        shooterCommand(TowerState.STOP);
        ringPusher.setPosition(RING_PUSHER_BACK);



        clawCommand(Claw.OPEN);

        towerAngle.setPosition(0.975);

        //motor mode's
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        //encoders initialization
        encoders.put("leftEncoder", leftRear); //2 E
        encoders.put("rightEncoder", leftFront); //1 E
        encoders.put("encoder", rightRear); //3 E
        encoders.put("wobble", manipulator); //0 C

        for (DcMotor dcMotor : encoders.values()) {
            dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void reset() {
        encoders.remove("wobble", shooter);
        for (DcMotor dcMotor : encoders.values()) {
            dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        encoders.put("wobble", shooter);
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

    public enum ManipulatorState {
        ASSEMBLED,
        DISASSEMBLED
    }

    public void manipulatorCommand(ManipulatorState control) {
        double SERVO_WOBBLE_OPEN = 0.65, SERVO_WOBBLE_CLOSE = 0.2;
        if (control == ManipulatorState.ASSEMBLED) manipulatorReturner.setPosition(SERVO_WOBBLE_OPEN);
        if (control == ManipulatorState.DISASSEMBLED) manipulatorReturner.setPosition(SERVO_WOBBLE_CLOSE);
    }
    /*
        public enum Door {
            OPEN,
            CLOSE
        }

        public void doorCommand (Door control) {
            if (control == Door.OPEN) {
                liftDoorLeft.setPosition(LIFT_DOOR_LEFT_OPEN);
                liftDoorRight.setPosition(LIFT_DOOR_RIGHT_OPEN);
            }
            if (control == Door.CLOSE) {
                liftDoorLeft.setPosition(LIFT_DOOR_LEFT_CLOSE);
                liftDoorRight.setPosition(LIFT_DOOR_RIGHT_CLOSE);
            }
        }

     */
    public enum Claw {
        OPEN,
        CLOSE
    }

    public void clawCommand(Claw control) {
        double CLAW_MIN = 0, CLAW_MAX = 1;
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
    }

    public enum PusherState {
        PUSHER_BACK,
        PUSHER_ON
    }

    public enum AngleState {
        TOSHOOT,
        TOINTAKE
    }

    public void shooterCommand(TowerState towerState) {
        if (towerState == TowerState.STOP) shooter.setVelocity(0);

        if (towerState == TowerState.SHOOTER_ON) shooter.setVelocity(3000);
    }

    public void pusherCommand(PusherState pusherState) {
        if (pusherState == PusherState.PUSHER_ON) {
            ringPusher.setPosition(RING_PUSHER_ON);
        }
        if(pusherState == PusherState.PUSHER_BACK){
            ringPusher.setPosition(RING_PUSHER_BACK);
        }
    }

    public void shooterAngleCommand(AngleState angleState){
        if(angleState == AngleState.TOINTAKE){
            towerAngle.setPosition(TOWER_ANGLE_MIN);
        }
        if(angleState == AngleState.TOSHOOT){
            towerAngle.setPosition(POSISHION_TO_SHOOT);
        }
    }
/*
    public void putLiftDown() {
        if (isLiftDown.isPressed()) ringLift.setPower(-0.6);
        else {
            ringLift.setPower(0);
            doorCommand(Door.OPEN);
            needLiftDown = false;
        }
    }


    public void putLiftUp(double speed) {
        pusherTimer.reset();
        if (isLiftUp.isPressed() && ringsIsNone.isPressed()) ringLift.setPower(speed);
        else if (!isLiftUp.isPressed() || !ringsIsNone.isPressed()){
            ringLift.setPower(0);
            needLiftUp = false;
        }
    }
*/

}
