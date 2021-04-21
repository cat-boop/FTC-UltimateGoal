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
import static org.firstinspires.ftc.teamcode.Hardware.PusherState;
import static org.firstinspires.ftc.teamcode.Hardware.ManipulatorState;

@TeleOp(name = "TeleOp")
public class TeleOperator extends LinearOpMode {

    Hardware robot = new Hardware();
    Gyroscope gyroscope = new Gyroscope();
    ElapsedTime wobbleTimer = new ElapsedTime(), shooterTimer = new ElapsedTime(),towerTimer = new ElapsedTime(), towerAngleTimer = new ElapsedTime(), sleepTimer = new ElapsedTime();

    Claw clawState = Claw.OPEN;
    TowerState towerState = TowerState.STOP;
    PusherState pusherState = Hardware.PusherState.PUSHER_BACK;

    PID wobblePID = new PID(0.01, 0, 0);
    int wobblePosition = 0;

    static final int DEBOUNCE_TIME = 300;

    double shooterLiftPosition = robot.getMinTowerAngle(), INCREMENT = 0.05;

    @Override
    public void runOpMode() {
        telemetry.addData("Status:", "Initializing");
        telemetry.update();

        robot.init(hardwareMap);
        gyroscope.init(hardwareMap);

        robot.manipulatorCommand(ManipulatorState.ASSEMBLED);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        waitForStart();

        sleepTimer.reset();
        wobbleTimer.reset();
        towerTimer.reset();
        towerAngleTimer.reset();

        while (!isStopRequested()) {

            //robot.shooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);

            //if (needStartShoot) robot.shoot();

            robot.shooterCommand(towerState);
            robot.pusherCommand(pusherState);

            firstGamepad(); //gamepad A

            secondGamepad(); //gamepad B

            telemetry.addData("shooter angle position", shooterLiftPosition);
            telemetry.addData("tower state", towerState);
            telemetry.addData("heading:", gyroscope.getAngle());
            telemetry.addData("encoder position", "%5d :%5d :%5d",
                    Objects.requireNonNull(encoders.get("encoder")).getCurrentPosition(),
                    Objects.requireNonNull(encoders.get("leftEncoder")).getCurrentPosition(),
                    Objects.requireNonNull(encoders.get("rightEncoder")).getCurrentPosition());
            telemetry.addData("wobble position", Objects.requireNonNull(encoders.get("wobble")).getCurrentPosition());
            telemetry.addData("shooter position", robot.shooter.getVelocity());
            telemetry.addData("pid coef", robot.shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.update();
        }
        robot.setPower(0, 0, 0);
    }


    public void firstGamepad() {
        robot.setPower(-gamepad1.right_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        //robot.setPower(-gamepad2.left_stick_y, -gamepad2.right_stick_x, -gamepad2.left_stick_x);

        if (gamepad1.x) clawState = Claw.OPEN; //claw
        if (gamepad1.y) clawState = Claw.CLOSE;
        robot.clawCommand(clawState);

        if (gamepad1.right_trigger > 0) { //manipulator
            robot.manipulator.setPower(gamepad1.right_trigger);
            wobblePosition = Objects.requireNonNull(encoders.get("wobble")).getCurrentPosition();
        }

        else if (gamepad1.left_trigger > 0) {
            robot.manipulator.setPower(-gamepad1.left_trigger);
            wobblePosition = Objects.requireNonNull(encoders.get("wobble")).getCurrentPosition();
        }
        else {
            if (clawState == Claw.CLOSE) robot.manipulator.setPower(-wobblePID.apply(Objects.requireNonNull(encoders.get("wobble")).getCurrentPosition() - wobblePosition));
            else {
                robot.manipulator.setPower(0);
                wobblePosition = Objects.requireNonNull(encoders.get("wobble")).getCurrentPosition();
            }
        }

    }

    public void secondGamepad() { //intake, shooter, pusher
        if (gamepad2.a){
            robot.pusherCommand(Hardware.PusherState.PUSHER_ON);
            sleep(500);
            robot.pusherCommand(Hardware.PusherState.PUSHER_BACK);
        }

        if (gamepad2.b && shooterTimer.milliseconds() > DEBOUNCE_TIME) {
            switch (towerState) {
                case STOP:
                    towerState = TowerState.SHOOTER_ON;
                    break;
                case SHOOTER_ON:
                    towerState = TowerState.STOP;
                    break;
            }
            shooterTimer.reset();
        }
        robot.shooterCommand(towerState);
        robot.clawCommand(clawState);

        if (gamepad2.right_trigger > 0) {
            //robot.intake.setPower(gamepad2.right_trigger);
            robot.intake.setPower(gamepad2.right_trigger);
            shooterLiftPosition = robot.getMaxTowerAngle();
        }
        else {
            robot.intake.setPower(-gamepad2.left_trigger);
        }

        if (gamepad2.dpad_up && towerAngleTimer.milliseconds() > DEBOUNCE_TIME) {
            shooterLiftPosition = Math.max(robot.getMinTowerAngle(), shooterLiftPosition - INCREMENT);
            towerAngleTimer.reset();
        }

        if (gamepad2.dpad_down && towerAngleTimer.milliseconds() > DEBOUNCE_TIME) {
            shooterLiftPosition = Math.min(robot.getMaxTowerAngle(), shooterLiftPosition + INCREMENT);
            towerAngleTimer.reset();
        }
        robot.towerAngle.setPosition(shooterLiftPosition);
    }

    public void sleep(int milliseconds) {
        double currentTime = sleepTimer.milliseconds();
        while (sleepTimer.milliseconds() - currentTime < milliseconds && opModeIsActive()) idle();
    }
}