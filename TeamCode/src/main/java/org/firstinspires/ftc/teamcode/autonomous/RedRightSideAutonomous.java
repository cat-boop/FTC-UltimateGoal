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

package org.firstinspires.ftc.teamcode.autonomous;

import java.lang.Math;
import java.util.Objects;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Gyroscope;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.PID;
import org.firstinspires.ftc.teamcode.vision.CameraHSV;

import static org.firstinspires.ftc.teamcode.Hardware.encoders;
import static org.firstinspires.ftc.teamcode.Hardware.Claw;
import static org.firstinspires.ftc.teamcode.Hardware.TowerState;
import static org.firstinspires.ftc.teamcode.Hardware.ManipulatorState;

@Autonomous(name="Red right side autonomous")
public class RedRightSideAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot   = new Hardware();   // Use a Pushbot's hardware
    Gyroscope gyroscope = new Gyroscope();
    CameraHSV camera = new CameraHSV();

    ElapsedTime sleepTimer = new ElapsedTime();
    ElapsedTime wobbleTimer = new ElapsedTime();


    PID wobblePID = new PID(0.01, 0, 0);

    static final double     INCHES_PER_TIC          = 0.0030326975625;
    static final double     WHEEL_DIAMETER_INCHES   = 1.1614173228346456692913385826772 * 2;
    static final double     DRIVE_SPEED             = 0.4;
    static final double     SIGN_X                  = -1;
    static final double     SIGN_ANGLE              = -1;

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


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");    //
        telemetry.update();

        robot.towerAngle.setPosition(0);

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
        sleep(200);

        wobblePosition = 0;


        driveByInches(DRIVE_SPEED, -8, 50);

        turnToAngle(15);

        shoot();

        if (isNone()) {
            wobblePosition = 50;
            turnToAngle( SIGN_ANGLE* 30);
            driveByInches(DRIVE_SPEED, 0, 16);

            robot.clawCommand(Claw.OPEN);
            sleep(300);

            driveByInches(DRIVE_SPEED, 0, -5);
            turnToAngle(0);
            driveByInches(DRIVE_SPEED, -SIGN_X * 15, 0);
            driveByInches(DRIVE_SPEED, 0, 15);
        }

        if (isOne()) {
            turnToAngle(0);

            wobblePosition = 60;
            driveByInches(DRIVE_SPEED, 0, 30);
            driveByInches(DRIVE_SPEED, 16,0);

            robot.clawCommand(Claw.OPEN);
            sleep(300);
            driveByInches(DRIVE_SPEED, 0, -5);

            driveByInches(DRIVE_SPEED,  -16,  0);
            driveByInches(DRIVE_SPEED, 0, -12);

        }

        if (isFour()) {
            wobblePosition = 50;
            turnToAngle(0);
            driveByInches(DRIVE_SPEED, 0, 60);

            turnToAngle(SIGN_ANGLE * 30);
            robot.clawCommand(Claw.OPEN);
            sleep(300);

            driveByInches(DRIVE_SPEED, 0, -7);
            turnToAngle(0);
            driveByInches(DRIVE_SPEED, 0, -50);
        }

    }

    public void returner() {
        double wobbleTime = wobbleTimer.milliseconds();

        while (wobbleTimer.milliseconds() - wobbleTime < 800 && opModeIsActive()) robot.manipulator.setPower(-0.5);
        robot.manipulator.setPower(0);

        robot.manipulatorCommand(ManipulatorState.ASSEMBLED);
        sleep(800);

        wobbleTimer.reset();
        double wobbleTime2 = wobbleTimer.milliseconds();

        while (wobbleTimer.milliseconds() - wobbleTime2 < 500 && opModeIsActive()) robot.manipulator.setPower(0.4);
        robot.manipulator.setPower(0);

        sleep(500);

        Objects.requireNonNull(encoders.get("wobble")).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Objects.requireNonNull(encoders.get("wobble")).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void sleep(int milliseconds) {
        double currentTime = sleepTimer.milliseconds();
        while (sleepTimer.milliseconds() - currentTime < milliseconds && opModeIsActive()) idle();
    }

    public boolean isNone() { return numberOfNone >= numberOfOne && numberOfNone >= numberOfFour; }
    public boolean isOne()  { return numberOfOne >= numberOfNone && numberOfOne >= numberOfFour; }
    public boolean isFour() { return numberOfFour >= numberOfNone && numberOfFour >= numberOfOne; }

    public void shootPowerShots(float firstAngle, float secondAngle, float thirdAngle ){

        robot.shooterCommand(TowerState.SHOOTER_ON);

        turnToAngle(firstAngle);
        sleep(1000);
        robot.pusherCommand(Hardware.PusherState.PUSHER_BACK);
        robot.pusherCommand(Hardware.PusherState.PUSHER_BACK);

        turnToAngle(secondAngle);
        sleep(1000);
        robot.pusherCommand(Hardware.PusherState.PUSHER_ON);
        robot.pusherCommand(Hardware.PusherState.PUSHER_BACK);

        turnToAngle(thirdAngle);
        sleep(1000);
        robot.pusherCommand(Hardware.PusherState.PUSHER_ON);
        robot.pusherCommand(Hardware.PusherState.PUSHER_BACK);

    }

    public void shoot(){
        robot.shooterCommand(TowerState.SHOOTER_ON);
        sleep(1000);
        for(int i=0; i<3; i++){
            robot.pusherCommand(Hardware.PusherState.PUSHER_ON);
            sleep(200);
            robot.pusherCommand(Hardware.PusherState.PUSHER_BACK);
            sleep(1500);
        }
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

        while (opModeIsActive()){

            regulateWobble();

            currentPositionX = Objects.requireNonNull(encoders.get("encoder")).getCurrentPosition();

            currentLeftPosition = Objects.requireNonNull(encoders.get("leftEncoder")).getCurrentPosition();
            currentRightPosition = Objects.requireNonNull(encoders.get("rightEncoder")).getCurrentPosition();

            arriveToX = Math.abs(currentPositionX) >= targetPositionX;
            arriveToY = Math.abs(currentLeftPosition) >= targetLeft; //|| Math.abs(currentRightPosition) >= targetRight;

            if (arriveToX && !arriveToY) robot.setPower(speed * signY, regulateAngle(angle), 0);
            if (arriveToY && !arriveToX) robot.setPower(0, regulateAngle(angle), speed * signX);

            if (arriveToX && arriveToY) break;
            // Display it for the driver.
            //telemetry.addData("Path1",  "Running to x%7d  y left%7d   y right%7d", targetPositionX, targetLeft);
            //telemetry.addData("Path2",  "Running at %7d %7d %7d", Math.abs(currentPositionX), Math.abs(currentLeftPosition));
            telemetry.addData("Running x:", targetPositionX);
            telemetry.addData("Running y:", targetLeft);
            telemetry.addData("angle", gyroscope.getAngle());
            telemetry.addData("wobble encoder", Objects.requireNonNull(encoders.get("wobble")).getCurrentPosition());
            telemetry.addData("wobble pid speed", wobblePID.apply(Objects.requireNonNull(encoders.get("wobble")).getCurrentPosition() - wobblePosition));
            telemetry.addData("currenPosition:", robot.manipulator.getCurrentPosition());
            telemetry.update();
        }
        robot.setPower(0, 0, 0);
        robot.reset();
    }

    public void turnToAngle(double targetAngle){
        angle = targetAngle;

        double currentAngle = gyroscope.getAngle();
        ElapsedTime timeAtTarget = new ElapsedTime();

        while ( (Math.abs(gyroscope.format(targetAngle, currentAngle)) > 5 || timeAtTarget.seconds() < 0.3) && opModeIsActive()) {

            regulateWobble();

            if (Math.abs(gyroscope.format(targetAngle, currentAngle)) > 7){
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
        robot.manipulator.setPower(-wobblePID.apply(Objects.requireNonNull(encoders.get("wobble")).getCurrentPosition() - wobblePosition));
    }
}
