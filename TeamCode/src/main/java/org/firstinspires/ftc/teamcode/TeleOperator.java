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
import static org.firstinspires.ftc.teamcode.Hardware.ManipulatorState;
import static org.firstinspires.ftc.teamcode.Hardware.needLiftDown;
import static org.firstinspires.ftc.teamcode.Hardware.needLiftUp;
import static org.firstinspires.ftc.teamcode.Hardware.needStartShoot;
import static org.firstinspires.ftc.teamcode.PIDFValues.kP;
import static org.firstinspires.ftc.teamcode.PIDFValues.kI;
import static org.firstinspires.ftc.teamcode.PIDFValues.kD;
import static org.firstinspires.ftc.teamcode.PIDFValues.kF;

@TeleOp(name = "TeleOp")
public class TeleOperator extends LinearOpMode {

    Hardware robot = new Hardware();
    Gyroscope gyroscope = new Gyroscope();
    ElapsedTime wobbleTimer = new ElapsedTime(), shooterTimer = new ElapsedTime(),towerTimer = new ElapsedTime(), towerAngleTimer = new ElapsedTime(), sleepTimer = new ElapsedTime();

    TowerState towerState = TowerState.STOP;
    Claw clawState = Claw.OPEN;
    Hardware.PusherState pusherState = Hardware.PusherState.PUSHER_BACK;


    PID wobblePID = new PID(0.005, 0, 0);
    int wobblePosition = 0;
    boolean canShoot = true;

    static final int DEBOUNCE_TIME = 300;

    double shooterLiftPosition = 1, INCREMENT = 0.025;

    /*
    if (gamepad1.a && towerTimer.milliseconds() > DEBOUNCE_TIME) {
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
    */

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


        //robot.clawCommand(clawState);

        if (gamepad1.right_trigger > 0) { //manipulator
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

    public void secondGamepad() { //intake, shooter, pusher
        //if (needLiftUp) robot.putLiftUp(0.8);
        //if (needLiftDown) robot.putLiftDown();
        //if (needStartShoot) robot.shoot();

        if (gamepad2.x ) clawState = Claw.OPEN; //claw
        if (gamepad2.y ) clawState = Claw.CLOSE;

        if (gamepad2.a){
            robot.pusherCommand(Hardware.PusherState.PUSHER_ON);
            sleep(500);
            robot.pusherCommand(Hardware.PusherState.PUSHER_BACK);
        }

        if (gamepad2.b && canShoot == true) {
            switch (towerState) {
                case STOP:
                    towerState = TowerState.SHOOTER_ON;
                    break;
                case SHOOTER_ON:
                    towerState = TowerState.STOP;
                    break;
            }
        }
        robot.shooterCommand(towerState);
        robot.clawCommand(clawState);

        /*
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
        }c


         */
        if (gamepad2.right_trigger > 0) {
            //robot.intake.setPower(gamepad2.right_trigger);
            robot.intake.setPower(gamepad2.right_trigger);
            canShoot = false;
        }
        else if(gamepad2.left_trigger >0){
            robot.intake.setPower(-gamepad2.left_trigger);
            canShoot = false;
        }
        else { canShoot = true;}



        if (gamepad2.dpad_up && towerAngleTimer.milliseconds() > DEBOUNCE_TIME) {
            shooterLiftPosition = Math.max(robot.getMinTowerAngle(), shooterLiftPosition - INCREMENT);

            if (gamepad2.dpad_up && towerAngleTimer.milliseconds() > DEBOUNCE_TIME) {
                shooterLiftPosition = 0.925;

                towerAngleTimer.reset();
            }

            if (gamepad2.dpad_down && towerAngleTimer.milliseconds() > DEBOUNCE_TIME) {
                shooterLiftPosition = Math.min(robot.getMaxTowerAngle(), shooterLiftPosition + INCREMENT);
                towerAngleTimer.reset();
            }
            robot.towerAngle.setPosition(shooterLiftPosition);


            // robot.ringLift.setPower(gamepad1.left_bumper ? -0.5 : 0);3

        }
    }

    /*
public void secondGamepad(){
    if (gamepad2.right_bumper) {
        //robot.intake.setPower(gamepad2.right_trigger);
        if (robot.isLiftDown.isPressed()) needLiftDown = true;
        else robot.intake.setPower(gamepad2.right_bumper ? 1 : 0);
    }
    else robot.intake.setPower(gamepad2.left_bumper ? -1 : 0);

    if (gamepad2.a && wobbleTimer.milliseconds() > DEBOUNCE_TIME) {
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
    if (gamepad2.x) clawState = Claw.OPEN;
    if (gamepad2.y) clawState = Claw.CLOSE;
    robot.clawCommand(clawState);

    if (gamepad2.right_trigger > 0) {
        robot.manipulator.setPower(gamepad2.right_trigger);
        wobblePosition = Objects.requireNonNull(encoders.get("wobble")).getCurrentPosition();
    }
    else if (gamepad2.left_trigger > 0) {
        robot.manipulator.setPower(-gamepad2.left_trigger);
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
*/
    public void sleep(int milliseconds) {
        double currentTime = sleepTimer.milliseconds();
        while (sleepTimer.milliseconds() - currentTime < milliseconds && opModeIsActive()) idle();
    }
}