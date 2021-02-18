package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

import static org.firstinspires.ftc.teamcode.Hardware.encoders;
import static org.firstinspires.ftc.teamcode.Hardware.Wobble;
import static org.firstinspires.ftc.teamcode.Hardware.TowerState;

@TeleOp(name = "TeleOp")
public class TeleOperator extends LinearOpMode {

    Hardware robot = new Hardware();
    Gyroscope gyroscope = new Gyroscope();
    ElapsedTime wobbleTimer = new ElapsedTime(), towerTimer = new ElapsedTime(), towerAngleTimer = new ElapsedTime();

    static final int DEBOUNCE_TIME = 200;

    double shooterLiftPosition = robot.TOWER_ANGLE_MAX, INCREMENT = 0.025;

    boolean needLiftDown = false, needLiftUp = false;

    TowerState towerState = TowerState.STOP;
    Wobble wobblePosition = Wobble.OPEN;

    @Override
    public void runOpMode() {
        telemetry.addData("Status:", "Initializing");
        telemetry.update();

        robot.init(hardwareMap);
        gyroscope.init(hardwareMap);

        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        waitForStart();

        wobbleTimer.reset();
        towerTimer.reset();
        towerAngleTimer.reset();

        while (!isStopRequested()) {
            firstGamepad();

            secondGamepad();

            telemetry.addData("shooter angle position", shooterLiftPosition);
            telemetry.addData("tower state", towerState);
            telemetry.addData("Heading:", gyroscope.getAngle());
            telemetry.addData("encoder position", "%5d :%5d :%5d",
                    Objects.requireNonNull(encoders.get("encoder")).getCurrentPosition(),
                    Objects.requireNonNull(encoders.get("leftEncoder")).getCurrentPosition(),
                    Objects.requireNonNull(encoders.get("rightEncoder")).getCurrentPosition());
            telemetry.update();
        }

        robot.setPower(0, 0, 0);
    }

    public void firstGamepad() {
        robot.setPower(-gamepad1.right_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        //robot.setPower(-gamepad2.left_stick_y, -gamepad2.right_stick_x, -gamepad2.left_stick_x);

        if (gamepad1.a && wobbleTimer.milliseconds() > DEBOUNCE_TIME) {
            switch (wobblePosition) {
                case OPEN:
                    wobblePosition = Wobble.CLOSE;
                    break;
                case CLOSE:
                    wobblePosition = Wobble.OPEN;
                    break;
            }
            robot.wobbleCommand(wobblePosition);
            wobbleTimer.reset();
        }

        if (gamepad1.x) robot.wobbleCommand(Wobble.OPEN);
        if (gamepad1.y) robot.wobbleCommand(Wobble.CLOSE);

        if (gamepad1.right_trigger > 0) robot.wobble.setPower(gamepad1.right_trigger);
        else robot.wobble.setPower(-gamepad1.left_trigger);
    }

    public void secondGamepad() {
        if (needLiftDown) putLiftDown(!robot.isLiftDown.isPressed());

        if (gamepad2.right_trigger > 0) {
            robot.intake.setPower(gamepad2.right_trigger);
            //if (robot.isLiftDown.isPressed()) needLiftDown = true;
            //else robot.intake.setPower(gamepad2.right_trigger);
        }
        else robot.intake.setPower(-gamepad2.left_trigger);


        if (gamepad2.dpad_left && towerAngleTimer.milliseconds() > DEBOUNCE_TIME) {
            shooterLiftPosition = Math.max(robot.TOWER_ANGLE_MIN, shooterLiftPosition - INCREMENT);
            towerAngleTimer.reset();
        }
        if (gamepad2.dpad_right && towerAngleTimer.milliseconds() > DEBOUNCE_TIME) {
            shooterLiftPosition = Math.min(robot.TOWER_ANGLE_MAX, shooterLiftPosition + INCREMENT);
            towerAngleTimer.reset();
        }
        robot.towerAngle.setPosition(shooterLiftPosition);


        if (gamepad2.dpad_up) robot.ringLift.setPower(0.5);
        else robot.ringLift.setPower(gamepad2.dpad_down ? -0.5 : 0);


        if (gamepad2.a && towerTimer.milliseconds() > DEBOUNCE_TIME) {
            switch (towerState) {
                case STOP:
                    towerState = TowerState.SHOOTER_ON;
                    break;
                case SHOOTER_ON:
                    towerState = TowerState.PUSHER_ON;
                    break;
                case PUSHER_ON:
                    towerState = TowerState.STOP;
                    break;
            }
            towerTimer.reset();
        }
        if (gamepad2.b) {
            towerState = TowerState.STOP;
        }
        robot.shooterCommand(towerState);
        robot.pusherCommand(towerState);
    }

    public void putLiftDown(boolean isLiftDown) {
        if (!isLiftDown) robot.ringLift.setPower(-0.3);
        else {
            robot.ringLift.setPower(0);
            needLiftDown = false;
        }
    }
}
