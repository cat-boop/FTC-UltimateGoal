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
import static org.firstinspires.ftc.teamcode.Hardware.Claw;

@Autonomous (name = "Blue side autonomous")
public class BlueSideAutonomous extends LinearOpMode {
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
        //camera.stop();

        returner();

        driveByInches(0.2, 0, 1);

        sleep(5000);

        robot.clawCommand(Claw.CLOSE);

        sleep(3000);

        driveByInches(DRIVE_SPEED, SIGN_X * 14, 0);

        if (isNone()) {
            wobblePosition = 100;
            driveByInches(DRIVE_SPEED, 0, 60);
            wobblePosition = 50;
            turnToAngle(SIGN_ANGLE * 30);

            //wobbleTimer = new ElapsedTime();
            //wobbleTimer.reset();
            //while (wobbleTimer.milliseconds() < 200) robot.wobble.setPower(0.3);

            robot.clawCommand(Hardware.Claw.OPEN);

            robot.shooterCommand(Hardware.TowerState.SHOOTER_ON);
            turnToAngle(0);
            driveByInches(DRIVE_SPEED, -SIGN_X * 24, -20);
            turnToAngle(-SIGN_ANGLE * 5);

            shoot();

            wobblePosition = 530;
            turnToAngle(0);
            driveByInches(0.4, SIGN_X * 2, -13);
            sleep(300);
            robot.clawCommand(Hardware.Claw.CLOSE);
            sleep(100);

            wobblePosition = 100;
            //turnToAngle(-205);
            turnToAngle(SIGN_ANGLE * 30);
            driveByInches(DRIVE_SPEED, 0, 40);

            robot.clawCommand(Hardware.Claw.OPEN);
            sleep(200);
            driveByInches(DRIVE_SPEED, 0, -4);
            driveByInches(DRIVE_SPEED, -SIGN_X * 13, 0);
        }

        if (isOne()) {
            wobblePosition = 100;

            robot.shooterCommand(Hardware.TowerState.SHOOTER_ON);
            driveByInches(DRIVE_SPEED, 0, 40);
            driveByInches(DRIVE_SPEED, -SIGN_X * 10, 10);
            wobblePosition = 0;

            turnToAngle(-SIGN_ANGLE * 10);
            shoot();
            turnToAngle(0);

            wobblePosition = 50;
            driveByInches(DRIVE_SPEED, 0, 30);

            robot.clawCommand(Hardware.Claw.OPEN);
            sleep(300);

            driveByInches(DRIVE_SPEED, 0, -10);
        }

        if (isFour()) {
            wobblePosition = 100;
            robot.shooterCommand(Hardware.TowerState.SHOOTER_ON);
            driveByInches(DRIVE_SPEED, 0, 40);
            driveByInches(DRIVE_SPEED, -SIGN_X * 10, 10);

            turnToAngle(-SIGN_ANGLE * 10);
            shoot();
            turnToAngle(0);

            wobblePosition = 50;
            driveByInches(DRIVE_SPEED, SIGN_X * 15, 55);
            turnToAngle(SIGN_ANGLE * 20);

            robot.clawCommand(Hardware.Claw.OPEN);

            turnToAngle(0);
            driveByInches(DRIVE_SPEED, 0, -40);
        }
    }

    public void returner() {
        robot.manipulatorCommand(Hardware.ManipulatorState.ASSEMBLED);
        sleep(400);

        wobbleTimer.reset();
        while (wobbleTimer.milliseconds() < 1000 && opModeIsActive()) robot.manipulator.setPower(0.8);
        robot.manipulator.setPower(0);

        sleep(5000);

        Objects.requireNonNull(encoders.get("wobble")).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Objects.requireNonNull(encoders.get("wobble")).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void sleep(int milliseconds) {
        double currentTime = sleepTimer.milliseconds();
        while (sleepTimer.milliseconds() - currentTime < milliseconds && opModeIsActive()) {
            telemetry.addData("past time", sleepTimer.milliseconds() - currentTime);
            telemetry.update();
        }
    }

    public boolean isNone() { return numberOfNone >= numberOfOne && numberOfNone >= numberOfFour; }
    public boolean isOne()  { return numberOfOne >= numberOfNone && numberOfOne >= numberOfFour; }
    public boolean isFour() { return numberOfFour >= numberOfNone && numberOfFour >= numberOfOne; }

    public void shoot() {
        while (opModeIsActive()) {
            robot.putLiftUp(0.2);
            if (!robot.ringsIsNone.isPressed()) break;
            robot.pusherCommand(Hardware.TowerState.PUSHER_ON);
            sleep(1000);
            //robot.pusherCommand(TowerState.STOP);
            //sleep(300);
        }
        robot.shooterCommand(Hardware.TowerState.STOP);
        robot.pusherCommand(Hardware.TowerState.STOP);
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

        while (opModeIsActive()) {

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

        while ( (Math.abs(gyroscope.format(targetAngle, currentAngle)) > 2 || timeAtTarget.seconds() < 0.5) && opModeIsActive()) {

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

