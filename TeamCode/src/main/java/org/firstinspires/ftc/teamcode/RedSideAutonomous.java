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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.vision.CameraHSV;

import static org.firstinspires.ftc.teamcode.Hardware.encoders;
import static org.firstinspires.ftc.teamcode.Hardware.Claw;
import static org.firstinspires.ftc.teamcode.Hardware.TowerState;

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
    static final double     DRIVE_SPEED             = 0.6;
    static final double     SIGN_X                  = -1;

    static final int        COUNTS_PER_MOTOR_REV    = 2400;
    static final double     COUNTS_PER_INCH         = COUNTS_PER_MOTOR_REV /
                                                    (WHEEL_DIAMETER_INCHES * 3.1415);

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

        robot.shoot();

        if (isNone()) {
            wobbleTimer.reset();
            //robot.wobble.setPower(0.7);

            driveByInches(DRIVE_SPEED, SIGN_X * 24, 72);
            robot.clawCommand(Claw.OPEN);
            sleep(200);

            wobbleTimer.reset();
            //robot.wobble.setPower(-0.5);
            driveByInches(DRIVE_SPEED, -SIGN_X * 30, -72);

            robot.clawCommand(Claw.CLOSE);
            sleep(200);

            wobbleTimer.reset();
            //robot.wobble.setPower(0.8);
            driveByInches(DRIVE_SPEED, SIGN_X * 30, 72);

            robot.clawCommand(Claw.OPEN);
            sleep(200);

            driveByInches(DRIVE_SPEED, 0, -5);
        }

        if (isOne()) {
            driveByInches(DRIVE_SPEED, 0, 96);
            robot.clawCommand(Claw.OPEN);
            driveByInches(DRIVE_SPEED, 0, -10);
        }

        if (isFour()){
            driveByInches(DRIVE_SPEED, SIGN_X * 24, 120);

        }

        /*
        if (numberOfNone >= numberOfOne && numberOfNone >= numberOfFour && opModeIsActive()) {
            telemetry.addData("Number of rings", 0);
            telemetry.update();

            //driveByTicks(DRIVE_SPEED, SIGN_X * 20, 5);
            driveByInches(DRIVE_SPEED, SIGN_X * 20, 5);

            robot.deployWobble();
            sleep(500);

            turnToAngle(180);

            //driveByTicks(DRIVE_SPEED, SIGN_X * 17, 20);
            //driveByTicks(DRIVE_SPEED, 0, 13);
            driveByInches(DRIVE_SPEED, SIGN_X * 18, 20);
            driveByInches(DRIVE_SPEED, 0, 13);

            sleep(1000); // grab wobble
            turnToAngle(0);

            //driveByTicks(DRIVE_SPEED, SIGN_X * 20, 24);
            //driveByTicks(DRIVE_SPEED, 0, 15);
            driveByInches(DRIVE_SPEED, SIGN_X * 20, 24);
            driveByInches(DRIVE_SPEED, 0, 15);

            driveByInches(DRIVE_SPEED, -SIGN_X * 20, 0);
            driveByInches(DRIVE_SPEED, 0, 5);
        }

        if (numberOfOne >= numberOfNone && numberOfOne >= numberOfFour && opModeIsActive()) {
            telemetry.addData("Number of rings", 1);
            telemetry.update();
            driveByInches(DRIVE_SPEED, 0, 43);

            robot.deployWobble();
            sleep(500);

            turnToAngle(180);

            driveByInches(DRIVE_SPEED, SIGN_X * 2.5, 67);

            turnToAngle(0);


            driveByInches(DRIVE_SPEED, 0, 60);

            driveByInches(DRIVE_SPEED, 0,-5);

        }

        if (numberOfFour >= numberOfNone && numberOfFour >= numberOfOne && opModeIsActive()) {
            telemetry.addData("Number of rings", 4);
            telemetry.update();
            driveByTicks(DRIVE_SPEED, SIGN_X * 10, 65);

            robot.deployWobble();
            sleep(500);

            driveByTicks(DRIVE_SPEED, 0, -40);
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
        //*/

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

    public void shoot(int numberOfRings) {
        robot.shooterCommand(TowerState.SHOOTER_ON);
        for (int i = 0; i < numberOfRings; i++) {
            robot.putLiftUp();
            robot.pusherCommand(TowerState.PUSHER_ON);
            sleep(500);
            robot.pusherCommand(TowerState.STOP);
            sleep(100);
        }
        robot.shooterCommand(TowerState.STOP);
    }

    public void driveByTicks(double speed, double x, double y) {
        int targetPositionX = Math.abs(encoders.get("encoder").getCurrentPosition() + (int) (x / INCHES_PER_TIC));

        int targetLeft = Math.abs(encoders.get("leftEncoder").getCurrentPosition() + (int) (y / INCHES_PER_TIC));
        int targetRight = Math.abs(encoders.get("rightEncoder").getCurrentPosition() + (int) (y / INCHES_PER_TIC));

        int signX = (x >= 0 ? 1 : -1);
        int signY = (y >= 0 ? 1 : -1);

        boolean arriveToX, arriveToY;

        robot.setPower(speed * signY, 0, speed * signX);

        while (opModeIsActive()) {
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
            telemetry.addData("op mode is active ticks", opModeIsActive());
            telemetry.update();
        }

        // Stop all motion
        robot.setPower(0, 0, 0);
        robot.reset();
    }

    public void driveByInches(double speed, double x, double y) {
        int targetPositionX = Math.abs(encoders.get("encoder").getCurrentPosition() + (int) (x * COUNTS_PER_INCH));

        int targetLeft = Math.abs(encoders.get("leftEncoder").getCurrentPosition() + (int) (y * COUNTS_PER_INCH));
        int targetRight = Math.abs(encoders.get("rightEncoder").getCurrentPosition() + (int) (y * COUNTS_PER_INCH));

        int signX = (x >= 0 ? 1 : -1);
        int signY = (y >= 0 ? 1 : -1);

        boolean arriveToX, arriveToY;

        robot.setPower(speed * signY, 0, speed * signX);

        while (opModeIsActive()) {
            if (wobbleTimer.seconds() > 2) robot.wobble.setPower(0);

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
            telemetry.addData("op mode is active inches", opModeIsActive());
            telemetry.update();
        }

        //if (isStopRequested()) camera.stop();
        // Stop all motion;
        robot.setPower(0, 0, 0);
        robot.reset();
    }

    public void turnToAngle(double targetAngle) {
        angle = targetAngle;

        double currentAngle = gyroscope.getAngle();
        ElapsedTime timeAtTarget = new ElapsedTime();

        while ( (Math.abs(gyroscope.format(targetAngle, currentAngle)) > 2 || timeAtTarget.seconds() < 0.5) && opModeIsActive()) {

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
            telemetry.addData("op mode is active", opModeIsActive());
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
}
