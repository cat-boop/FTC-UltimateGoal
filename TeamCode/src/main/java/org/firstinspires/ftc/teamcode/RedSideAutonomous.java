/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import java.lang.Math;
import java.util.Objects;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.vision.CameraHSV;

import static org.firstinspires.ftc.teamcode.Hardware.encoders;
import static org.firstinspires.ftc.teamcode.Hardware.Claw;
import static org.firstinspires.ftc.teamcode.Hardware.TowerState;
import static org.firstinspires.ftc.teamcode.Hardware.needLiftUp;
import static org.firstinspires.ftc.teamcode.Hardware.needStartShoot;

@Autonomous(name="Red side autonomous")
public class RedSideAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot   = new Hardware();   // Use a Pushbot's hardware
    Gyroscope gyroscope = new Gyroscope();
    CameraHSV camera = new CameraHSV();

    ElapsedTime time = new ElapsedTime();
    ElapsedTime wobbleTimer = new ElapsedTime();

    PID wobblePID = new PID(0.05, 0, 0);

    static final double     INCHES_PER_TIC          = 0.0030326975625;
    static final double     WHEEL_DIAMETER_INCHES   = 1.1614173228346456692913385826772 * 2;
    static final double     DRIVE_SPEED             = 0.8;
    static final double     SIGN_X                  = -1;

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
        msStuckDetectStop = 2500;
        robot.init(hardwareMap);
        gyroscope.init(hardwareMap);
        camera.init(hardwareMap, telemetry);

        //robot.clawCommand(Claw.CLOSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");    //
        telemetry.update();

        waitForStart();

        for (int i = 0; i < 500 && opModeIsActive(); i++) {
            int number = camera.getNumberOfRing();
            if (number == 0) numberOfNone++;
            if (number == 1) numberOfOne++;
            if (number == 4) numberOfFour++;
        }
        camera.stop();

        zalom();

        driveByInches(DRIVE_SPEED, SIGN_X * 14, 0, false);
        driveByInches(DRIVE_SPEED, 0, 2, false);
        sleep(300);
        robot.clawCommand(Claw.CLOSE);
        sleep(300);

        //robot.shoot();

        if (isNone()) {
            wobblePosition = 200;
            driveByInches(DRIVE_SPEED, 0, 60, true);
            wobblePosition = 0;
            turnToAngle(-30, true);

            wobbleTimer = new ElapsedTime();
            wobbleTimer.reset();
            while (wobbleTimer.milliseconds() < 200) robot.wobble.setPower(0.3);

            robot.clawCommand(Claw.OPEN);

            robot.shooterCommand(TowerState.SHOOTER_ON);
            turnToAngle(0, false);
            driveByInches(DRIVE_SPEED, -SIGN_X * 24, -20, false);
            turnToAngle(5, false);

            shoot();

            wobblePosition = 500;
            turnToAngle(0, true);
            driveByInches(DRIVE_SPEED, SIGN_X * 3, -13, true);
            sleep(300);
            robot.clawCommand(Claw.CLOSE);
            sleep(100);

            wobblePosition = 100;
            turnToAngle(-205, true);
            driveByInches(DRIVE_SPEED, 0, -45, true);

            //ElapsedTime wobbleTimer = new ElapsedTime();
            //wobbleTimer.reset();
            //while (wobbleTimer.milliseconds() < 1000) regulateWobble();

            robot.clawCommand(Claw.OPEN);
            sleep(200);
            driveByInches(DRIVE_SPEED, SIGN_X * 10, 0, false);
            driveByInches(DRIVE_SPEED, 0, 0, false);
        }

        if (isOne()) {
            wobblePosition = 100;

            robot.shooterCommand(TowerState.SHOOTER_ON);
            driveByInches(DRIVE_SPEED, 0, 40, true);
            driveByInches(DRIVE_SPEED, -SIGN_X * 10, 10, true);
            wobblePosition = 0;

            turnToAngle(10, false);
            shoot();
            turnToAngle(0, true);

            driveByInches(DRIVE_SPEED, 0, 30, true);

            wobbleTimer = new ElapsedTime();
            wobbleTimer.reset();
            while (wobbleTimer.milliseconds() < 400) robot.wobble.setPower(0.3);

            robot.clawCommand(Claw.OPEN);
            sleep(500);

            driveByInches(DRIVE_SPEED, 0, -10, false);
        }

        if (isFour()){
            wobblePosition = 100;
            robot.shooterCommand(TowerState.SHOOTER_ON);
            driveByInches(DRIVE_SPEED, 0, 40, true);
            driveByInches(DRIVE_SPEED, -SIGN_X * 10, 10, true);

            robot.wobble.setPower(0);
            turnToAngle(10, false);
            shoot();
            turnToAngle(0, true);

            driveByInches(DRIVE_SPEED, SIGN_X * 15, 55, false);
            turnToAngle(-20, false);

            wobbleTimer = new ElapsedTime();
            wobbleTimer.reset();
            while (wobbleTimer.milliseconds() < 200) robot.wobble.setPower(0.3);
            robot.clawCommand(Claw.OPEN);

            turnToAngle(0, false);
            driveByInches(DRIVE_SPEED, 0, -24, false);
            driveByInches(DRIVE_SPEED, -SIGN_X * 10, -24, false);
        }
    }

    public void zalom() {

        wobbleTimer.reset();
        while (wobbleTimer.milliseconds() < 300) robot.wobble.setPower(0.3);
//        wobblePosition = -345;
//        wobbleTimer.reset();
//        while(Math.abs(encoders.get("wobble").getCurrentPosition()) < Math.abs(wobblePosition) && opModeIsActive()) {
//            robot.wobble.setPower(0.4);
//            telemetry.addData("wobble encoder", encoders.get("wobble").getCurrentPosition());
//            telemetry.update();
//        }
//
//        sleep(500);
//
//        wobblePosition = -10;
//        wobbleTimer.reset();
//        while(Math.abs(encoders.get("wobble").getCurrentPosition()) > Math.abs(wobblePosition) && opModeIsActive()) {
//            robot.wobble.setPower(-1);
//            telemetry.addData("wobble encoder", encoders.get("wobble").getCurrentPosition());
//            telemetry.update();
//        }
//
//        sleep(1000);
//
//        wobblePosition = -340;
//        wobbleTimer.reset();
//        while(wobbleTimer.milliseconds() < 500 && opModeIsActive()) {
//            regulateWobble();
//        }

        encoders.get("wobble").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoders.get("wobble").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public boolean isNone() { return numberOfNone >= numberOfOne && numberOfNone >= numberOfFour && opModeIsActive(); }
    public boolean isOne()  { return numberOfOne >= numberOfNone && numberOfOne >= numberOfFour && opModeIsActive(); }
    public boolean isFour() { return numberOfFour >= numberOfNone && numberOfFour >= numberOfOne && opModeIsActive(); }

    public void rotateWobble() {
        time.reset();
        double currentTime = time.seconds();
        while (time.seconds() - currentTime < 1) robot.wobble.setPower(1);
        robot.wobble.setPower(0);
        sleep(500);
        robot.clawCommand(Claw.CLOSE);
        sleep(500);
    }

    public void shoot() {
        while (opModeIsActive()) {
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

    public void driveByInches(double speed, double x, double y, boolean needRegulateWobble) {
        int targetPositionX = Math.abs(encoders.get("encoder").getCurrentPosition() + (int) (x * COUNTS_PER_INCH));

        int targetLeft = Math.abs(encoders.get("leftEncoder").getCurrentPosition() + (int) (y * COUNTS_PER_INCH));
        int targetRight = Math.abs(encoders.get("rightEncoder").getCurrentPosition() + (int) (y * COUNTS_PER_INCH));

        int signX = (x >= 0 ? 1 : -1);
        int signY = (y >= 0 ? 1 : -1);

        boolean arriveToX, arriveToY;

        robot.setPower(speed * signY, 0, speed * signX);

        while (opModeIsActive()) {
            if (needRegulateWobble) regulateWobble();

            arriveToX = Math.abs(encoders.get("encoder").getCurrentPosition()) >= targetPositionX;
            arriveToY = Math.abs(encoders.get("leftEncoder").getCurrentPosition()) >= targetLeft ||
                    Math.abs(encoders.get("rightEncoder").getCurrentPosition()) >= targetRight;

            if (arriveToX && !arriveToY) robot.setPower(speed * signY, regulateAngle(angle), 0);
            if (arriveToY && !arriveToX) robot.setPower(0, regulateAngle(angle), speed * signX);

            if (arriveToX && arriveToY) break;
            // Display it for the driver.
            telemetry.addData("Path1",  "Running to x%7d  y left%7d   y right%7d", targetPositionX, targetLeft, targetRight);
            telemetry.addData("Path2",  "Running at %7d %7d %7d",
                    Math.abs(encoders.get("encoder").getCurrentPosition()),
                    Math.abs(encoders.get("rightEncoder").getCurrentPosition()),
                    Math.abs(encoders.get("leftEncoder").getCurrentPosition()));
            telemetry.addData("angle", gyroscope.getAngle());
            telemetry.addData("speed forward", speed * signY);
            telemetry.addData("speed strafe", speed * signX);
            telemetry.addData("wobble pid speed", wobblePID.apply(Objects.requireNonNull(encoders.get("wobble")).getCurrentPosition() - wobblePosition));
            telemetry.update();
        }

        //if (isStopRequested()) camera.stop();
        // Stop all motion;
        robot.setPower(0, 0, 0);
        robot.reset();
    }

    public void turnToAngle(double targetAngle, boolean needRegulateWobble) {
        angle = targetAngle;

        double currentAngle = gyroscope.getAngle();
        ElapsedTime timeAtTarget = new ElapsedTime();

        while ( (Math.abs(gyroscope.format(targetAngle, currentAngle)) > 2 || timeAtTarget.seconds() < 0.5) && opModeIsActive()) {

            if (needRegulateWobble) regulateWobble();

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
        robot.wobble.setPower(wobblePID.apply(Objects.requireNonNull(encoders.get("wobble")).getCurrentPosition() - wobblePosition));
    }
}
