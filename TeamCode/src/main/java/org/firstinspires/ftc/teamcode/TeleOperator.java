package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

import static org.firstinspires.ftc.teamcode.Hardware.encoders;
import static org.firstinspires.ftc.teamcode.Hardware.Claw;
import static org.firstinspires.ftc.teamcode.Hardware.TowerState;
import static org.firstinspires.ftc.teamcode.Hardware.Door;
import static org.firstinspires.ftc.teamcode.Hardware.ManipulatorState;
import static org.firstinspires.ftc.teamcode.Hardware.needLiftDown;
import static org.firstinspires.ftc.teamcode.Hardware.needLiftUp;
import static org.firstinspires.ftc.teamcode.Hardware.needStartShoot;

@Config
@TeleOp(name = "TeleOp")
public class TeleOperator extends LinearOpMode {

    Hardware robot = new Hardware();
    Gyroscope gyroscope = new Gyroscope();
    ElapsedTime wobbleTimer = new ElapsedTime(), towerTimer = new ElapsedTime(), towerAngleTimer = new ElapsedTime();

    TowerState towerState = TowerState.STOP;
    Claw clawState = Claw.OPEN;

    PID wobblePID = new PID(0.005, 0, 0);
    int wobblePosition = 0;

    static final int DEBOUNCE_TIME = 300;

    double shooterLiftPosition = 1, INCREMENT = 0.025;

    @Override
    public void runOpMode() {
        telemetry.addData("Status:", "Initializing");
        telemetry.update();

        robot.init(hardwareMap);
        gyroscope.init(hardwareMap);

        robot.manipulatorCommand(ManipulatorState.ASSEMBLED);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

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
            telemetry.addData("heading:", gyroscope.getAngle());
            telemetry.addData("encoder position", "%5d :%5d :%5d",
                    Objects.requireNonNull(encoders.get("encoder")).getCurrentPosition(),
                    Objects.requireNonNull(encoders.get("leftEncoder")).getCurrentPosition(),
                    Objects.requireNonNull(encoders.get("rightEncoder")).getCurrentPosition());
            telemetry.addData("wobble position", Objects.requireNonNull(encoders.get("wobble")).getCurrentPosition());
            telemetry.addData("shooter position", robot.shooter.getVelocity());
            telemetry.addData("have ring button pressed", !robot.isLiftUp.isPressed());
            telemetry.addData("none ring button pressed", !robot.ringsIsNone.isPressed());
            telemetry.addData("pid coef", robot.shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.update();
        }

        robot.setPower(0, 0, 0);
    }

    public void firstGamepad() {
        robot.setPower(-gamepad1.right_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        //robot.setPower(-gamepad2.left_stick_y, -gamepad2.right_stick_x, -gamepad2.left_stick_x);

        if (gamepad1.a && wobbleTimer.milliseconds() > DEBOUNCE_TIME) {
            switch (clawState) {
                case OPEN:
                    clawState = Claw.CLOSE;
                    break;
                case CLOSE:
                    clawState = Claw.OPEN;
                    break;
            }
            wobbleTimer.reset();
        }
        if (gamepad1.x) clawState = Claw.OPEN;
        if (gamepad1.y) clawState = Claw.CLOSE;
        robot.clawCommand(clawState);

        if (gamepad1.right_trigger > 0) {
            robot.manipulator.setPower(gamepad1.right_trigger);
            wobblePosition = Objects.requireNonNull(encoders.get("wobble")).getCurrentPosition();
        }
        else if (gamepad1.left_trigger > 0) {
            robot.manipulator.setPower(-gamepad1.left_trigger);
            wobblePosition = Objects.requireNonNull(encoders.get("wobble")).getCurrentPosition();
        }
        else {
            if (clawState == Claw.CLOSE) robot.manipulator.setPower(wobblePID.apply(Objects.requireNonNull(encoders.get("wobble")).getCurrentPosition() - wobblePosition));
            else {
                robot.manipulator.setPower(0);
                wobblePosition = Objects.requireNonNull(encoders.get("wobble")).getCurrentPosition();
            }
        }

    }

    public void secondGamepad() {
        if (needLiftUp) robot.putLiftUp(0.8);
        if (needLiftDown) robot.putLiftDown();
        if (needStartShoot) robot.shoot();

        if (gamepad2.right_trigger > 0) {
            //robot.intake.setPower(gamepad2.right_trigger);
            if (robot.isLiftDown.isPressed()) needLiftDown = true;
            else robot.intake.setPower(gamepad2.right_trigger);
        }
        else robot.intake.setPower(-gamepad2.left_trigger);


        if (gamepad2.dpad_left && towerAngleTimer.milliseconds() > DEBOUNCE_TIME) {
            shooterLiftPosition = Math.max(robot.getMinTowerAngle(), shooterLiftPosition - INCREMENT);
            towerAngleTimer.reset();
        }
        if (gamepad2.dpad_right && towerAngleTimer.milliseconds() > DEBOUNCE_TIME) {
            shooterLiftPosition = Math.min(robot.getMaxTowerAngle(), shooterLiftPosition + INCREMENT);
            towerAngleTimer.reset();
        }
        robot.towerAngle.setPosition(shooterLiftPosition);


        if (gamepad2.dpad_up) {
            needLiftUp = true;
            needLiftDown = false;
            robot.ringLift.setPower(0);
            robot.doorCommand(Door.CLOSE);
            sleep(300);
            robot.doorCommand(Door.OPEN);
            sleep(150);
        }
        if (gamepad2.dpad_down) {
            needLiftDown = true;
            needLiftUp = false;
            robot.ringLift.setPower(0);
        }

        if (gamepad2.right_bumper) robot.ringLift.setPower(0.5);
        else if (!needLiftUp && !needLiftDown) robot.ringLift.setPower(gamepad2.left_bumper ? -0.5 : 0);

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

        if (gamepad2.x && !needLiftDown) {
            robot.doorCommand(Door.OPEN);
            sleep(300);
        }
        if (gamepad2.y && !needLiftDown) {
            robot.doorCommand(Door.CLOSE);
            sleep(300);
        }

        robot.shooterCommand(towerState);
        robot.pusherCommand(towerState);
    }
}
