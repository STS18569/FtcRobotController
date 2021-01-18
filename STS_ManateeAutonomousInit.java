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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="Manatee: Autonomous Init and Test", group="UltimateGoal")
//@Disabled
public class STS_ManateeAutonomousInit extends LinearOpMode {

    /* Declare OpMode members. */
    STS_HardwareManatee     manatee = new STS_HardwareManatee();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 28.0;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 4.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
    static final double     FUDGE_FACTOR            = 23.0 / 44.0;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * FUDGE_FACTOR) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        manatee.init(hardwareMap);

        manatee.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        manatee.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        manatee.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        manatee.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        manatee.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        manatee.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        manatee.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        manatee.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d",
                manatee.leftFrontDrive.getCurrentPosition(),
                manatee.leftBackDrive.getCurrentPosition(),
                manatee.rightFrontDrive.getCurrentPosition(),
                manatee.rightBackDrive.getCurrentPosition());


        waitForStart();
        testMotors();
        testServos();

    }

    public void testMotors() {
        encoderDrive(DRIVE_SPEED,  -48,  -48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
    }

    public void testServos() {
        manatee.wobbleArm.setPosition(1.0);
        telemetry.addData("Test Servos", "wobbleArm.setPosition: %.3f", manatee.wobbleArm.getPosition());
        telemetry.update();
        sleep( 2000);     // pause for servos to move
        /*

        manatee.wobbleClaw.setPosition(0.1);
        telemetry.addData("Test Servos", "wobbleClaw.setPosition: %.3f", manatee.wobbleClaw.getPosition());
        telemetry.update();
        sleep( 2000);     // pause for servos to move

        manatee.wobbleClaw.setPosition(0.5);
        telemetry.addData("Test Servos", "wobbleClaw.setPosition: %.3f", manatee.wobbleClaw.getPosition());
        telemetry.update();
        sleep( 2000);     // pause for servos to move

        manatee.shooterAngler.setPosition(1.0);
        telemetry.addData("Test Servos", "shooterAngler.setPosition: %.3f", manatee.shooterAngler.getPosition());
        telemetry.update();
        sleep( 3000);     // pause for servos to move

        manatee.shooterAngler.setPosition(0.5);
        telemetry.addData("Test Servos", "shooterAngler.setPosition: %.3f", manatee.shooterAngler.getPosition());
        telemetry.update();
        sleep( 3000);     // pause for servos to move

        manatee.shooterAngler.setPosition(0.0);
        telemetry.addData("Test Servos", "shooterAngler.setPosition: %.3f", manatee.shooterAngler.getPosition());
        telemetry.update();
        sleep( 3000);     // pause for servos to move
        
         */

        telemetry.addData("Test Servos", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = manatee.leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftBackTarget = manatee.leftBackDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = manatee.rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightBackTarget = manatee.rightBackDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            manatee.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            manatee.leftBackDrive.setTargetPosition(newLeftBackTarget);
            manatee.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            manatee.rightBackDrive.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            manatee.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            manatee.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            manatee.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            manatee.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            manatee.leftFrontDrive.setPower(Math.abs(speed));
            manatee.leftBackDrive.setPower(Math.abs(speed));
            manatee.rightFrontDrive.setPower(Math.abs(speed));
            manatee.rightBackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (manatee.leftFrontDrive.isBusy() && manatee.leftBackDrive.isBusy() && manatee.rightFrontDrive.isBusy() && manatee.rightBackDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newLeftFrontTarget,  newLeftBackTarget,  newRightFrontTarget,  newRightBackTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                                            manatee.leftFrontDrive.getCurrentPosition(),
                                            manatee.leftBackDrive.getCurrentPosition(),
                                            manatee.rightFrontDrive.getCurrentPosition(),
                                            manatee.rightBackDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            manatee.leftFrontDrive.setPower(0);
            manatee.leftBackDrive.setPower(0);
            manatee.rightFrontDrive.setPower(0);
            manatee.rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            manatee.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            manatee.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            manatee.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            manatee.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
