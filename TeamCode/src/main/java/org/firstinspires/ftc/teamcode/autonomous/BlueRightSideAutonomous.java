package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Gyroscope;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.PID;
import org.firstinspires.ftc.teamcode.vision.CameraHSV;

import java.util.Objects;

import static org.firstinspires.ftc.teamcode.Hardware.encoders;
import static org.firstinspires.ftc.teamcode.Hardware.ManipulatorState;
import static org.firstinspires.ftc.teamcode.Hardware.TowerState;
import static org.firstinspires.ftc.teamcode.Hardware.Claw;

@Autonomous (name = "Blue right side autonomous")
public class BlueRightSideAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot   = new Hardware();   // Use a Pushbot's hardware
    Gyroscope gyroscope = new Gyroscope();
    CameraHSV camera = new CameraHSV();

    ElapsedTime sleepTimer = new ElapsedTime();
    ElapsedTime wobbleTimer = new ElapsedTime();

    PID wobblePID = new PID(0.01, 0, 0);

    static final double     INCHES_PER_TIC          = 0.0030326975625;
    static final double     WHEEL_DIAMETER_INCHES   = 1.1614173228346456692913385826772 * 2;
    static final double     DRIVE_SPEED             = 0.7;
    static final double     SIGN_X                  = 1;
    static final double     SIGN_ANGLE              = 1;

    static final int        COUNTS_PER_MOTOR_REV    = 2400;
    static final double     COUNTS_PER_INCH         = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * 3.1415);

    int wobblePosition;
    int numberOfNone = 0, numberOfOne = 0, numberOfFour = 0;
    double angle = 0;

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        //msStuckDetectStop = 1000;
        robot.init(hardwareMap);
        gyroscope.init(hardwareMap);
        camera.init(hardwareMap, telemetry);

        //robot.clawCommand(Claw.CLOSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");    //
        telemetry.update();

        waitForStart();

        sleepTimer.reset();
        wobbleTimer.reset();

        for (int i = 0; i < 500 && opModeIsActive(); i++) {
            int number = camera.getNumberOfRing();
            if (number == 0) numberOfNone++;
            if (number == 1) numberOfOne++;
            if (number == 4) numberOfFour++;
        }
        camera.stop();

        returner();

        robot.clawCommand(Claw.CLOSE);
        sleep(500);

        wobblePosition = 60;
        robot.shooterCommand(TowerState.SHOOTER_ON);
        driveByInches(DRIVE_SPEED, -SIGN_X * 10, 55 - 2);

        shootPowerShot();

        if (isNone()) {
            driveByInches(DRIVE_SPEED, 0, 16);
            turnToAngle(SIGN_ANGLE * 90);

            wobblePosition = 100;
            driveByInches(DRIVE_SPEED, 0, 33);

            robot.clawCommand(Claw.OPEN);
            sleep(200);

            driveByInches(DRIVE_SPEED, 0, -40 + 12);
        }

        if (isOne()) {
            driveByInches(DRIVE_SPEED, 0, 16 + 24);
            turnToAngle(SIGN_ANGLE * 90);

            wobblePosition = 50;
            driveByInches(DRIVE_SPEED, 0, 20);

            robot.clawCommand(Claw.OPEN);
            sleep(200);

            driveByInches(DRIVE_SPEED, 0, -25);
            turnToAngle(0);

            driveByInches(DRIVE_SPEED, 0, -16 - 24);
        }

        if (isFour()) {
            driveByInches(DRIVE_SPEED, 0, 16 + 48);
            turnToAngle(SIGN_ANGLE * 90);

            wobblePosition = 50;
            driveByInches(DRIVE_SPEED, 0, 50);

            robot.clawCommand(Claw.OPEN);
            sleep(200);

            driveByInches(DRIVE_SPEED, 0, -50);
            turnToAngle(0);

            driveByInches(DRIVE_SPEED, 0, -16 - 48);
        }
        // */
    }

    public void returner() {
        robot.manipulatorCommand(ManipulatorState.ASSEMBLED);
        sleep(500);

        double wobbleTime = wobbleTimer.milliseconds();
        while (wobbleTimer.milliseconds() - wobbleTime < 1000 && !isStopRequested()) robot.manipulator.setPower(0.8);
        robot.manipulator.setPower(0);

        sleep(500);

        Objects.requireNonNull(encoders.get("wobble")).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Objects.requireNonNull(encoders.get("wobble")).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void sleep(int milliseconds) {
        double currentTime = sleepTimer.milliseconds();
        while (sleepTimer.milliseconds() - currentTime < milliseconds && !isStopRequested()) {
            robot.manipulator.setPower(0);
        }
    }

    public boolean isNone() { return numberOfNone >= numberOfOne && numberOfNone >= numberOfFour && !isStopRequested(); }
    public boolean isOne()  { return numberOfOne >= numberOfNone && numberOfOne >= numberOfFour && !isStopRequested(); }
    public boolean isFour() { return numberOfFour >= numberOfNone && numberOfFour >= numberOfOne && !isStopRequested(); }

    public void shoot() {
        double shootTime = sleepTimer.milliseconds();
        while (opModeIsActive() && sleepTimer.milliseconds() - shootTime < 3000) {
            robot.putLiftUp(0.2);
            if (!robot.ringsIsNone.isPressed()) break;
            robot.pusherCommand(TowerState.PUSHER_ON);
            sleep(1000);
            //robot.pusherCommand(TowerState.STOP);
            //sleep(300);
        }
        robot.shooterCommand(TowerState.STOP);
        robot.pusherCommand(TowerState.STOP);
    }

    public void shootPowerShot() {
        robot.shooterCommand(TowerState.SHOOTER_ON);
        sleep(500);
        for (int i = 0; i < 3 && !isStopRequested(); i++) {
            double shootTime = sleepTimer.milliseconds();
            while (robot.isLiftUp.isPressed() && robot.ringsIsNone.isPressed() && sleepTimer.milliseconds() - shootTime < 2000) {
                robot.putLiftUp(0.3);
            }
            robot.pusherCommand(TowerState.PUSHER_ON);

            if (i != 2) {
                robot.ringLift.setPower(-0.1);
                sleep(100);
                robot.ringLift.setPower(0);
            }

            sleep(1500);
            robot.pusherCommand(TowerState.STOP);
            if (i != 2) turnToAngle(-SIGN_ANGLE * 5 * (i + 1));
        }
        turnToAngle(0);
        robot.shooterCommand(TowerState.STOP);
    }

    public void driveByInches(double speed, double x, double y) {
        int currentPositionX = Objects.requireNonNull(encoders.get("encoder")).getCurrentPosition();

        int currentLeftPosition = Objects.requireNonNull(encoders.get("leftEncoder")).getCurrentPosition();
        int currentRightPosition = Objects.requireNonNull(encoders.get("rightEncoder")).getCurrentPosition();

        int targetPositionX = Math.abs(currentPositionX + (int) (x * COUNTS_PER_INCH));

        int targetLeft  = Math.abs(currentLeftPosition + (int) (y * COUNTS_PER_INCH));
        int targetRight = Math.abs(currentRightPosition + (int) (y * COUNTS_PER_INCH));

        int signX = (x >= 0 ? 1 : -1);
        int signY = (y >= 0 ? 1 : -1);

        boolean arriveToX, arriveToY;

        robot.setPower(speed * signY, 0, speed * signX);

        while (!isStopRequested()) {

            regulateWobble();

            currentPositionX = Objects.requireNonNull(encoders.get("encoder")).getCurrentPosition();

            currentLeftPosition = Objects.requireNonNull(encoders.get("leftEncoder")).getCurrentPosition();
            currentRightPosition = Objects.requireNonNull(encoders.get("rightEncoder")).getCurrentPosition();

            arriveToX = Math.abs(currentPositionX) >= targetPositionX;
            arriveToY = Math.abs(currentLeftPosition) >= targetLeft || Math.abs(currentRightPosition) >= targetRight;

            if (arriveToX && !arriveToY) robot.setPower(speed * signY, regulateAngle(angle), 0);
            if (arriveToY && !arriveToX) robot.setPower(0, regulateAngle(angle), speed * signX);

            if (arriveToX && arriveToY) break;
            // Display it for the driver.
            telemetry.addData("Path1",  "Running to x%7d  y left%7d   y right%7d", targetPositionX, targetLeft, targetRight);
            telemetry.addData("Path2",  "Running at %7d %7d %7d", Math.abs(currentPositionX), Math.abs(currentLeftPosition), Math.abs(currentRightPosition));
            telemetry.addData("angle", gyroscope.getAngle());
            telemetry.addData("wobble encoder", Objects.requireNonNull(encoders.get("wobble")).getCurrentPosition());
            telemetry.addData("wobble pid speed", wobblePID.apply(Objects.requireNonNull(encoders.get("wobble")).getCurrentPosition() - wobblePosition));
            telemetry.update();
        }
        robot.setPower(0, 0, 0);
        robot.reset();
    }

    public void turnToAngle(double targetAngle) {
        angle = targetAngle;

        double currentAngle = gyroscope.getAngle();
        ElapsedTime timeAtTarget = new ElapsedTime();

        while ( (Math.abs(gyroscope.format(targetAngle, currentAngle)) > 2 || timeAtTarget.seconds() < 0.5) && !isStopRequested()) {

            regulateWobble();

            if (Math.abs(gyroscope.format(targetAngle, currentAngle)) > 4) {
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

    public void regulateWobble() {
        robot.manipulator.setPower(wobblePID.apply(Objects.requireNonNull(encoders.get("wobble")).getCurrentPosition() - wobblePosition));
    }
}

