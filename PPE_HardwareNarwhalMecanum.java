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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryInternal;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeServices;

import java.util.concurrent.TimeUnit;
*/

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class PPE_HardwareNarwhalMecanum extends PPE_HardwareNarwhalChassis
{

    public DcMotor  leftFrontDrive      = null;
    public DcMotor  leftBackDrive       = null;
    public DcMotor  rightFrontDrive     = null;
    public DcMotor  rightBackDrive      = null;

    /* Constructor */
    public PPE_HardwareNarwhalMecanum(PPE_HardwareNarwhalChassis.driverMode hwDriverMode, Telemetry telemetry) {
        super(hwDriverMode, telemetry);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);

        // Define and Initialize Motors
        leftFrontDrive = hwMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hwMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hwMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void doLoop(Gamepad gamepad1)
    {
        super.doLoop(gamepad1);

        double rotationLeft = gamepad1.right_stick_x;
        double rotationRight = -gamepad1.right_stick_x;

        double r = (Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y)) * (-1);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4.0;
        double rightX = gamepad1.right_stick_x;

        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        if ((rotationLeft != 0) || (rotationRight != 0)) {
            if (rotationLeft != 0) {
                telemetry.addLine("ROTATION LEFT MODE");
                leftFrontDrive.setPower(WHEEL_SPEED_MULTIPLIER * rotationLeft);
                rightFrontDrive.setPower(WHEEL_SPEED_MULTIPLIER * -rotationLeft);
                leftBackDrive.setPower(WHEEL_SPEED_MULTIPLIER * rotationLeft);
                rightBackDrive.setPower(WHEEL_SPEED_MULTIPLIER * -rotationLeft);
            }
            else {
                telemetry.addLine("ROTATION RIGHT MODE");
                leftFrontDrive.setPower(WHEEL_SPEED_MULTIPLIER * -rotationRight);
                rightFrontDrive.setPower(WHEEL_SPEED_MULTIPLIER * rotationRight);
                leftBackDrive.setPower(WHEEL_SPEED_MULTIPLIER * -rotationRight);
                rightBackDrive.setPower(WHEEL_SPEED_MULTIPLIER * rotationRight);
            }
        }
        else {
            telemetry.addLine("LATERAL MODE");
            leftFrontDrive.setPower(WHEEL_TURN_SPEED_MULTIPLIER * v2);
            rightFrontDrive.setPower(WHEEL_TURN_SPEED_MULTIPLIER * v1);
            leftBackDrive.setPower(WHEEL_TURN_SPEED_MULTIPLIER * v4);
            rightBackDrive.setPower(WHEEL_TURN_SPEED_MULTIPLIER * v3);
        }

        telemetry.addData("leftFrontDrive.Power", "%.2f", leftFrontDrive.getPower());
        telemetry.addData("rightFrontDrive.Power", "%.2f", rightFrontDrive.getPower());
        telemetry.addData("leftBackDrive.Power", "%.2f", leftBackDrive.getPower());
        telemetry.addData("rightBackDrive.Power", "%.2f", rightBackDrive.getPower());
        telemetry.update();
    }

    public void encoderDrive(double speed, double degree,
                             double leftInches, double rightInches,
                             double timeoutS, PPE_HardwareNarwhalChassis.DriveMode mode, LinearOpMode curLinearOpMode) {
        super.encoderDrive(speed, degree, leftInches, rightInches, timeoutS, mode, curLinearOpMode);

        if (degree < 0) {
            leftInches = (degree*((2 * WHEEL_BASE * Math.PI)/360));
            rightInches = -((degree*((2 * WHEEL_BASE * Math.PI)/360)));
        }
        else if (degree > 0) {
            rightInches = (degree*((2 * WHEEL_BASE * Math.PI)/360));
            leftInches = -((degree*((2 * WHEEL_BASE * Math.PI)/360)));
        }
        else if (degree == 0) {
            leftInches = leftInches;
            rightInches = rightInches;
        }

        int newLeftFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightFrontTarget = 0;
        int newRightBackTarget = 0;

        switch(mode) {
            case LINEAR:
                newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                break;
            case LAT_RIGHT:
                newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(-rightInches * COUNTS_PER_INCH);
                newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(-rightInches * COUNTS_PER_INCH);
                newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                break;
            case LAT_LEFT:
                newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(-leftInches * COUNTS_PER_INCH);
                newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(-leftInches * COUNTS_PER_INCH);
                break;
            default:
                // code block
        }
        
        // Ensure that the opmode is still active
        if (curLinearOpMode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            switch(mode) {
                case LINEAR:
                    newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                    newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                    newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                    newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                    break;
                case LAT_RIGHT:
                    newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                    newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(-rightInches * COUNTS_PER_INCH);
                    newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(-rightInches * COUNTS_PER_INCH);
                    newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                    break;
                case LAT_LEFT:
                    newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(-leftInches * COUNTS_PER_INCH);
                    newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                    newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                    newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(-leftInches * COUNTS_PER_INCH);
                    break;
                default:
                    // code block
            }

            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            //sleep(0100);
            rightFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            // telemetry.addData("speed == ", speed);
            // telemetry.addData("timeoutS == ", timeoutS);
            while (curLinearOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    ((leftFrontDrive.isBusy() || rightFrontDrive.isBusy()))) {

                // Display it for the driver.
                /*
                telemetry.addData("speed",  speed);
                telemetry.addData("degree",  degree);
                telemetry.addData("leftInches",  leftInches);
                telemetry.addData("rightInches",  rightInches);
                telemetry.addData("timeoutS",  timeoutS);
                 */
                telemetry.addData("Path1 (target)",  "Running to %7d :%7d :%7d :%7d", newLeftFrontTarget,  newLeftBackTarget,
                        newRightFrontTarget,  newRightBackTarget);
                telemetry.addData("Path2 (position)",  "Running at %7d :%7d :%7d :%7d", leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                // telemetry.addData("EncoderDrive", "time(%3d) : %3f", timeoutS, runtime.seconds());
                telemetry.addData("EncoderDrive: time: ", runtime.seconds());
                telemetry.update();
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    //TODO: TESTING TIMEEEE
    public void encoderDriveTest1(double speed, double degree,
                                 double leftInches, double rightInches,
                                 double timeoutS, PPE_HardwareNarwhalChassis.DriveMode mode, LinearOpMode curLinearOpMode) {
    super.encoderDrive(speed, degree, leftInches, rightInches, timeoutS, mode, curLinearOpMode);
    // Declares variables that are used for this method

    if (degree < 0) {
        leftInches = degree*((WHEEL_BASE * Math.PI)/360);
        rightInches = -((degree*((WHEEL_BASE * Math.PI)/360)));
    }

    else if (degree > 0) {
        rightInches = degree*((WHEEL_BASE * Math.PI)/360);
        leftInches = -((degree*((WHEEL_BASE * Math.PI)/360)));
    }

    else if (degree == 0) {
        leftInches = leftInches;
        rightInches = rightInches;
    }

    int newLeftFrontTarget;
    int newLeftBackTarget;
    int newRightFrontTarget;
    int newRightBackTarget;
    int rightPosition;
    int leftPosition;
    double leftPower;
    double rightPower;

    // Resets encoders to 0
    leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    if (curLinearOpMode.opModeIsActive()) {
        switch(mode) {
            case LINEAR:
        // Determine new target position, and pass to motor controller
        // Calculates the needed encoder ticks by multiplying a pre-determined amount of CountsPerInches,
        // and the method input gets the actual distance travel in inches
        newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        newRightBackTarget = rightBackDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

                // Gets the current position of the encoders at the beginning of the EncoderDrive method
        /*
        rightPosition = rightDrive.getCurrentPosition();
        leftPosition = leftDrive.getCurrentPosition();
         */

                leftFrontDrive.setTargetPosition(newLeftFrontTarget);
                leftBackDrive.setTargetPosition(newLeftBackTarget);
                rightFrontDrive.setTargetPosition(newRightFrontTarget);
                rightBackDrive.setTargetPosition(newRightBackTarget);


        // Turn On RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // reset the timeout time and start motion.
        runtime.reset();
        leftPower = speed;
        rightPower = speed;
        leftFrontDrive.setPower(leftPower);
        leftBackDrive.setPower(leftPower);
        rightFrontDrive.setPower(rightPower);
        rightBackDrive.setPower(rightPower);
        boolean Running = true;
            /*
            double LeftEncoderPositionAtFullSpeed;
            double RightEncoderPositionAtFullSpeed;
            boolean Running;
            // This gets where the motor encoders will be at full position when it will be at full speed.
            if ((leftInches > 3) && (rightInches > 3)) {
                LeftEncoderPositionAtFullSpeed = ((3 * (COUNTS_PER_INCH)) + LeftPosition);
                RightEncoderPositionAtFullSpeed = ((3 * (COUNTS_PER_INCH)) + RightPosition);
                Running = true;
            } else {
                LeftEncoderPositionAtFullSpeed = (LeftPosition);
                RightEncoderPositionAtFullSpeed = (RightPosition);
                Running = true;
            }

             */

        // This gets the absolute value of the encoder positions at full speed - the current speed, and while it's greater than 0, it will continues increasing the speed.
        // This allows the robot to accelerate over a set number of inches, which reduces wheel slippage and increases overall reliability
        while ((leftFrontDrive.getTargetPosition() != leftFrontDrive.getCurrentPosition()) && (rightFrontDrive.getTargetPosition() != rightFrontDrive.getCurrentPosition()) &&
                Running && curLinearOpMode.opModeIsActive() && (runtime.seconds() < timeoutS)) {

            telemetry.addData("Working", "moving");
            telemetry.update();

            if (((Math.abs(leftFrontDrive.getTargetPosition()) - Math.abs(leftFrontDrive.getCurrentPosition())) <= (4 * COUNTS_PER_INCH)) && ((Math.abs(leftFrontDrive.getPower()) > .15))
            && (leftFrontDrive.getTargetPosition() != leftFrontDrive.getCurrentPosition()) && (rightFrontDrive.getTargetPosition() != rightFrontDrive.getCurrentPosition())) {
                // This allows the robot to accelerate over a set distance, rather than going full speed.  This reduces wheel slippage and increases reliability.

                telemetry.addData("Working", "slowing");
                telemetry.update();

                leftPower = (Range.clip(Math.abs((leftFrontDrive.getPower()) * 0.5), .15, speed));
                rightPower = (Range.clip(Math.abs((rightBackDrive.getPower()) * 0.5), .15, speed));

                leftFrontDrive.setPower(leftPower);
                leftBackDrive.setPower(leftPower);
                rightFrontDrive.setPower(rightPower);
                rightBackDrive.setPower(rightPower);

                //telemetry.addData("In Accel loop CM", + Distance.getDistance(DistanceUnit.CM));
                telemetry.addData("Current Power", "%.2f", leftPower);
                telemetry.addData("Path1 (target)", "Running to %7d :%7d :%7d :%7d", newLeftFrontTarget, newLeftBackTarget,
                        newRightFrontTarget, newRightBackTarget);
                telemetry.addData("Path2 (position)", "Running at %7d :%7d :%7d :%7d", leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                //telemetry.addData("Sections Complete:", +SectionsCompleted);
                telemetry.update();
            } else if ((leftFrontDrive.getTargetPosition() == leftFrontDrive.getCurrentPosition()) && (rightFrontDrive.getTargetPosition() == rightFrontDrive.getCurrentPosition())) {
                Running = false;
                /*
                //This if statement might be kinda pointless... leaving it in for now
            } else if (((Math.abs(newLeftFrontTarget) - Math.abs(leftFrontDrive.getCurrentPosition()) < COUNTS_PER_INCH) || (Math.abs(newLeftFrontTarget) - Math.abs(leftBackDrive.getCurrentPosition()) > COUNTS_PER_INCH))
                    && ((Math.abs(newLeftBackTarget) - Math.abs(leftBackDrive.getCurrentPosition()) < COUNTS_PER_INCH) || (Math.abs(newLeftBackTarget) - Math.abs(leftBackDrive.getCurrentPosition()) > COUNTS_PER_INCH))
                    && ((Math.abs(newRightFrontTarget) - Math.abs(rightFrontDrive.getCurrentPosition()) < COUNTS_PER_INCH) || (Math.abs(newRightFrontTarget) - Math.abs(rightFrontDrive.getCurrentPosition()) > COUNTS_PER_INCH))
                    && ((Math.abs(newRightBackTarget) - Math.abs(rightBackDrive.getCurrentPosition()) < COUNTS_PER_INCH) || (Math.abs(newRightBackTarget) - Math.abs(rightBackDrive.getCurrentPosition()) > COUNTS_PER_INCH))) {
                Running = false;
                 */

            } else {

                // Multiplies the speed desired by the direction, which has a value of either 1, or -1, and allows for backwards driving with the ramp up function
                leftFrontDrive.setPower(speed);
                leftBackDrive.setPower(speed);
                rightFrontDrive.setPower(speed);
                rightBackDrive.setPower(speed);

                //telemetry.addData("In Reg loop CM", + Distance.getDistance(DistanceUnit.CM));
                telemetry.addData("Path1 (target)",  "Running to %7d :%7d :%7d :%7d", newLeftFrontTarget,  newLeftBackTarget,
                        newRightFrontTarget,  newRightBackTarget);
                telemetry.addData("Path2 (position)",  "Running at %7d :%7d :%7d :%7d", leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                //telemetry.addData("Sections Complete:", +SectionsCompleted);
                telemetry.update();


            }

            // Display information for the driver.

        }
                break;
            case LAT_LEFT:
                newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(-rightInches * COUNTS_PER_INCH);
                newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(-rightInches * COUNTS_PER_INCH);
                newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);

            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            //sleep(0100);
            rightFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            // telemetry.addData("speed == ", speed);
            // telemetry.addData("timeoutS == ", timeoutS);
            while (curLinearOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    ((leftFrontDrive.isBusy() || rightFrontDrive.isBusy()))) {

                // Display it for the driver.
                /*
                telemetry.addData("speed",  speed);
                telemetry.addData("degree",  degree);
                telemetry.addData("leftInches",  leftInches);
                telemetry.addData("rightInches",  rightInches);
                telemetry.addData("timeoutS",  timeoutS);
                 */
                telemetry.addData("Path1 (target)",  "Running to %7d :%7d :%7d :%7d", newLeftFrontTarget,  newLeftBackTarget,
                        newRightFrontTarget,  newRightBackTarget);
                telemetry.addData("Path2 (position)",  "Running at %7d :%7d :%7d :%7d", leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                // telemetry.addData("EncoderDrive", "time(%3d) : %3f", timeoutS, runtime.seconds());
                telemetry.addData("EncoderDrive: time: ", runtime.seconds());
                telemetry.update();
            }
            break;

            case LAT_RIGHT:
                newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(-leftInches * COUNTS_PER_INCH);
                newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(-leftInches * COUNTS_PER_INCH);

            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            //sleep(0100);
            rightFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            // telemetry.addData("speed == ", speed);
            // telemetry.addData("timeoutS == ", timeoutS);
            while (curLinearOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    ((leftFrontDrive.isBusy() || rightFrontDrive.isBusy()))) {

                // Display it for the driver.
                /*
                telemetry.addData("speed",  speed);
                telemetry.addData("degree",  degree);
                telemetry.addData("leftInches",  leftInches);
                telemetry.addData("rightInches",  rightInches);
                telemetry.addData("timeoutS",  timeoutS);
                 */
                telemetry.addData("Path1 (target)",  "Running to %7d :%7d :%7d :%7d", newLeftFrontTarget,  newLeftBackTarget,
                        newRightFrontTarget,  newRightBackTarget);
                telemetry.addData("Path2 (position)",  "Running at %7d :%7d :%7d :%7d", leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                // telemetry.addData("EncoderDrive", "time(%3d) : %3f", timeoutS, runtime.seconds());
                telemetry.addData("EncoderDrive: time: ", runtime.seconds());
                telemetry.update();
            }
            break;
            default:
        }

        telemetry.addData("Working", "end");
        telemetry.update();

        // Stops all motion
        // Set to run without encoder, so it's not necessary to declare this every time after the method is used
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Set power to 0
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

    }
}
}



