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
    ElapsedTime wobbleTimer = new ElapsedTime(), towerAngleTimer2 = new ElapsedTime(), gradesTimer = new ElapsedTime(), shooterTimer = new ElapsedTime(),towerTimer = new ElapsedTime(), towerAngleTimer = new ElapsedTime(), sleepTimer = new ElapsedTime();

    Claw clawState = Claw.OPEN;
    TowerState towerState = TowerState.STOP;
    PusherState pusherState = Hardware.PusherState.PUSHER_BACK;

    //double  kP = 25, kI =0 , kD=7, kF=13;
    static final double SIGN_ANGLE = -1;

    PID wobblePID = new PID(0.01, 0, 0);
    int wobblePosition = 0;

    static final int DEBOUNCE_TIME = 300;
    public boolean canMovePusher = false;
    public  boolean canIntake = true;

    //double shooterVelocity , INCREMENTSHOOTER = 50;

    double shooterLiftPosition = robot.getMinTowerAngle(), INCREMENT = 0.10;
    double angle = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status:", "Initializing");
        telemetry.update();



        robot.init(hardwareMap);
        gyroscope.init(hardwareMap);

        //robot.manipulatorCommand(ManipulatorState.ASSEMBLED);

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
            //telemetry.addData("target velocity", shooterVelocity);
            telemetry.addData("pid coef", robot.shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.addData("canIntake:", canIntake);
            telemetry.addData("TowerAngle", robot.towerAngle.getPosition());
            telemetry.update();
        }
        robot.setPower(0, 0, 0);
    }


    public void firstGamepad() {
        //robot.setPower(-gamepad1.right_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        robot.setPower(-gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x);

        if (gamepad1.x) clawState = Claw.OPEN; //claw
        if (gamepad1.y) clawState = Claw.CLOSE;
        robot.clawCommand(clawState);

        if (gamepad1.dpad_left && gradesTimer.milliseconds() > DEBOUNCE_TIME) {
            turnToAngleTeleOp();
        }

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

        /*
        if (gamepad2.dpad_left && towerAngleTimer.milliseconds() > DEBOUNCE_TIME) {
            shooterLiftPosition = Math.max(robot.getMinTowerAngle(), shooterLiftPosition - INCREMENT);
            towerAngleTimer.reset();
            towerAngleTimer.reset();
        }

        if (gamepad2.dpad_right && towerAngleTimer.milliseconds() > DEBOUNCE_TIME) {
            shooterLiftPosition = Math.min(robot.getMaxTowerAngle(), shooterLiftPosition + INCREMENT);
            towerAngleTimer.reset();
        }

         */

                if (gamepad2.dpad_up && towerAngleTimer.milliseconds() > DEBOUNCE_TIME) {
            //shooterLiftPosition = Math.max(robot.getMinTowerAngle(), shooterLiftPosition - INCREMENT);
            //towerAngleTimer.reset();
            robot.towerAngle.setPosition(0);
            canIntake = false;
            towerAngleTimer.reset();
        }

        if (gamepad2.dpad_down && towerAngleTimer.milliseconds() > DEBOUNCE_TIME) {
            //shooterLiftPosition = Math.min(robot.getMaxTowerAngle(), shooterLiftPosition + INCREMENT);
            //towerAngleTimer.reset();
            robot.towerAngle.setPosition(0.6);
            canIntake = true;
            towerAngleTimer.reset();
        }


        if (gamepad2.b ){
            robot.pusherCommand(Hardware.PusherState.PUSHER_ON);
            sleep(150);
            robot.pusherCommand(Hardware.PusherState.PUSHER_BACK);
        }


        if (gamepad2.a && shooterTimer.milliseconds() > DEBOUNCE_TIME) {
            switch (towerState) {
                case STOP:
                    towerState = TowerState.SHOOTER_ON;
                    canMovePusher = true;
                    break;
                case SHOOTER_ON:
                    towerState = TowerState.STOP;
                    canMovePusher = false;
                    break;
            }
            shooterTimer.reset();
        }
        robot.shooterCommand(towerState);
        robot.clawCommand(clawState);



        if (gamepad2.right_trigger > 0) {
            robot.towerAngle.setPosition(0.6);
            //robot.intake.setPower(gamepad2.right_trigger);
            robot.intake.setPower(gamepad2.right_trigger);
        }
        else {
            robot.intake.setPower(-gamepad2.left_trigger);
        }

       //  robot.towerAngle.setPosition(shooterLiftPosition);

    }

    public void turnToAngleTeleOp() {

        double currentAngle = gyroscope.getAngle();
        ElapsedTime timeAtTarget = new ElapsedTime();

        double targetAngle = currentAngle + 180;

        while ( (Math.abs(gyroscope.format(targetAngle, currentAngle)) > 4 || timeAtTarget.milliseconds() <= 300) && opModeIsActive()) {


            if (Math.abs(gyroscope.format(targetAngle, currentAngle)) > 8) {
                timeAtTarget.reset();
            }

            double power = gyroscope.turnTo(currentAngle, targetAngle);
            currentAngle = gyroscope.getAngle();
            robot.setPower(0, power, 0);

            telemetry.addData("Current angle", currentAngle);
            telemetry.addData("Time at target", timeAtTarget.seconds());
            telemetry.addData("Error", gyroscope.format(targetAngle, currentAngle));
            telemetry.addData("Power", power);
            telemetry.addData("wobble pid speed", wobblePID.apply(Objects.requireNonNull(encoders.get("wobble")).getCurrentPosition() - wobblePosition));
            telemetry.update();
        }
        robot.setPower(0, 0, 0);
        robot.reset();
    }

    public double regulateAngle(double targetAngle) {
        double currentAngle = gyroscope.getAngle();
        double power = gyroscope.turnTo(currentAngle, targetAngle);
        if (Math.abs(Math.abs(targetAngle) - Math.abs(currentAngle)) < 1.5) return 0;
        return power;
    }

    public void sleep(int milliseconds) {
        double currentTime = sleepTimer.milliseconds();
        while (sleepTimer.milliseconds() - currentTime < milliseconds && opModeIsActive()) idle();
    }
}