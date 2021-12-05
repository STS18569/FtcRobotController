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
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//@Autonomous(name="Narwhal: Autonomous Init", group="FreightFrenzy")
//@Disabled
public abstract class PPE_NarwhalAutonomousInit extends LinearOpMode {

    /* Declare OpMode members. */
    PPE_HardwareNarwhal narwhalHW = new PPE_HardwareNarwhalExternals2022();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static enum             DriveMode {LAT_LEFT, LAT_RIGHT, LINEAR};

    static final double COUNTS_PER_MOTOR_REV = 28.0;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 4.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.5;     // For figuring circumference
    static final double WHEEL_BASE = 10.8;
    static final double FUDGE_FACTOR = 0.89;
    static final double COUNTS_PER_INCH = 22.48;
    static final double DRIVE_SPEED = 0.3;
    static final double LATERAL_ADJUSTMENT = 1.0;
    static final double TURN_SPEED = 0.09;
    static final double REAL_TURN_SPEED = 0.35;
    static final double TURN_FUDGE_FACTOR = 0.75;

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        narwhalHW.init(hardwareMap);

        narwhalHW.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        narwhalHW.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        narwhalHW.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        narwhalHW.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                narwhalHW.leftDrive.getCurrentPosition(),
                narwhalHW.rightDrive.getCurrentPosition());


        waitForStart();

        runAutonomousMode();
    }

    public abstract void runAutonomousMode();

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

    //TODO: ENCODERDRIVE HIGHLY UNRELIABLE, OFTEN OVERSHOT OR UNDERSHOT TURN
    public void encoderDriveOmni(double speed, double degree,
                                 double leftInches, double rightInches,
                                 double timeoutS) {
        if (degree < 0) {
            leftInches = -(degree * ((WHEEL_BASE * Math.PI) / 360));
            rightInches = ((degree * ((WHEEL_BASE * Math.PI) / 360)));
        } else if (degree > 0) {
            leftInches = -((degree * ((WHEEL_BASE * Math.PI) / 360)));
            rightInches = degree * ((WHEEL_BASE * Math.PI) / 360);
        } else if (degree == 0) {
            leftInches = leftInches;
            rightInches = rightInches;
        }

        int newLeftTarget = 0;
        int newRightTarget = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller

            newLeftTarget = narwhalHW.leftDrive.getCurrentPosition() + (int) (-leftInches * COUNTS_PER_INCH);
            newRightTarget = narwhalHW.rightDrive.getCurrentPosition() + (int) (-rightInches * COUNTS_PER_INCH);
            // code block

            narwhalHW.leftDrive.setTargetPosition(newLeftTarget);
            narwhalHW.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            narwhalHW.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            narwhalHW.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            narwhalHW.leftDrive.setPower(Math.abs(speed));
            narwhalHW.rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            // telemetry.addData("speed == ", speed);
            // telemetry.addData("timeoutS == ", timeoutS);
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    ((narwhalHW.leftDrive.isBusy() || narwhalHW.rightDrive.isBusy()))) {

                // Display it for the driver.
                /*
                telemetry.addData("speed",  speed);
                telemetry.addData("degree",  degree);
                telemetry.addData("leftInches",  leftInches);
                telemetry.addData("rightInches",  rightInches);
                telemetry.addData("timeoutS",  timeoutS);
                 */
                telemetry.addData("Path1 (target)", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2 (position)", "Running at %7d :%7d", narwhalHW.leftDrive.getCurrentPosition(), narwhalHW.rightDrive.getCurrentPosition());
                // telemetry.addData("EncoderDrive", "time(%3d) : %3f", timeoutS, runtime.seconds());
                telemetry.addData("EncoderDrive: time: ", runtime.seconds());
                telemetry.update();
            }

            // Stop all motion;
            narwhalHW.leftDrive.setPower(0);
            narwhalHW.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            narwhalHW.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            narwhalHW.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderDriveMecanum(DriveMode mode, double speed, double degree,
                             double leftInches, double rightInches,
                             double timeoutS) {

        if (degree < 0) {
            leftInches = degree*((WHEEL_BASE * Math.PI)/360);
            rightInches = -(TURN_FUDGE_FACTOR *(degree*((WHEEL_BASE * Math.PI)/360)));
        }

        else if (degree > 0) {
            rightInches = degree*((WHEEL_BASE * Math.PI)/360);
            leftInches = -(TURN_FUDGE_FACTOR *(degree*((WHEEL_BASE * Math.PI)/360)));
        }

        else if (degree == 0) {
            leftInches = leftInches;
            rightInches = rightInches;
        }

        int newLeftFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightFrontTarget = 0;
        int newRightBackTarget = 0;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            switch(mode) {
                case LINEAR:
                    newLeftFrontTarget = narwhalHW.leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                    newLeftBackTarget = narwhalHW.leftBackDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                    newRightFrontTarget = narwhalHW.rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                    newRightBackTarget = narwhalHW.rightBackDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                    break;
                case LAT_LEFT:
                    newLeftFrontTarget = narwhalHW.leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                    newLeftBackTarget = narwhalHW.leftBackDrive.getCurrentPosition() + (int)(-rightInches * COUNTS_PER_INCH);
                    newRightFrontTarget = narwhalHW.rightFrontDrive.getCurrentPosition() + (int)(-rightInches * COUNTS_PER_INCH);
                    newRightBackTarget = narwhalHW.rightBackDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                    break;
                case LAT_RIGHT:
                    newLeftFrontTarget = narwhalHW.leftFrontDrive.getCurrentPosition() + (int)(-leftInches * COUNTS_PER_INCH);
                    newLeftBackTarget = narwhalHW.leftBackDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                    newRightFrontTarget = narwhalHW.rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                    newRightBackTarget = narwhalHW.rightBackDrive.getCurrentPosition() + (int)(-leftInches * COUNTS_PER_INCH);
                    break;
                default:
                    // code block
            }

            narwhalHW.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            narwhalHW.leftBackDrive.setTargetPosition(newLeftBackTarget);
            narwhalHW.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            narwhalHW.rightBackDrive.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            narwhalHW.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            narwhalHW.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            narwhalHW.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            narwhalHW.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            narwhalHW.leftFrontDrive.setPower(Math.abs(speed));
            narwhalHW.leftBackDrive.setPower(Math.abs(speed));
            //sleep(0100);
            narwhalHW.rightFrontDrive.setPower(Math.abs(speed));
            narwhalHW.rightBackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            // telemetry.addData("speed == ", speed);
            // telemetry.addData("timeoutS == ", timeoutS);
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    ((narwhalHW.leftFrontDrive.isBusy() || narwhalHW.rightFrontDrive.isBusy()))) {

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
                telemetry.addData("Path2 (position)",  "Running at %7d :%7d :%7d :%7d", narwhalHW.leftFrontDrive.getCurrentPosition(), narwhalHW.leftBackDrive.getCurrentPosition(),
                        narwhalHW.rightFrontDrive.getCurrentPosition(), narwhalHW.rightBackDrive.getCurrentPosition());
                // telemetry.addData("EncoderDrive", "time(%3d) : %3f", timeoutS, runtime.seconds());
                telemetry.addData("EncoderDrive: time: ", runtime.seconds());
                telemetry.update();
            }

            // Stop all motion;
            narwhalHW.leftFrontDrive.setPower(0);
            narwhalHW.leftBackDrive.setPower(0);
            narwhalHW.rightFrontDrive.setPower(0);
            narwhalHW.rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            narwhalHW.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            narwhalHW.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            narwhalHW.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            narwhalHW.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
