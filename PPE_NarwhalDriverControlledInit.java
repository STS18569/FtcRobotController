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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * This file provides basic Teleop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Narwhal: Driver Controlled Init", group="FreightFrenzy")
// @Disabled
public class PPE_NarwhalDriverControlledInit extends OpMode{
    /* Declare OpMode members. */
    public PPE_HardwareNarwhal narwhalHW;

    boolean             carouselIsMoving = false;
    boolean             armIsMovingForward = false;
    boolean             armIsMovingBackward = false;
    boolean             intakeIsMoving = false;

    double              armLidOffset = 0.0;
    double              ARM_LID_SPEED = 0.0025;

    final double        WHEEL_SPEED_MULTIPLIER = 1.0;

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException var4) {
            Thread.currentThread().interrupt();
        }
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        narwhalHW = new PPE_HardwareNarwhalExternals2022(); // use the class created to define a STS_HardwareManatee's hardware
        narwhalHW.init(hardwareMap);

        narwhalHW.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        narwhalHW.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("BREAKING:", "BatBoy Initialized");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left = gamepad1.left_stick_y;
        double right = gamepad1.right_stick_y;
        double rotationLeft = gamepad1.right_stick_x;
        double rotationRight = - gamepad1.right_stick_x;


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
                narwhalHW.leftFrontDrive.setPower(WHEEL_SPEED_MULTIPLIER * rotationLeft);
                narwhalHW.rightFrontDrive.setPower(WHEEL_SPEED_MULTIPLIER * -rotationLeft);
                narwhalHW.leftBackDrive.setPower(WHEEL_SPEED_MULTIPLIER * rotationLeft);
                narwhalHW.rightBackDrive.setPower(WHEEL_SPEED_MULTIPLIER * -rotationLeft);
            }
            else {
                telemetry.addLine("ROTATION RIGHT MODE");
                narwhalHW.leftFrontDrive.setPower(WHEEL_SPEED_MULTIPLIER * -rotationRight);
                narwhalHW.rightFrontDrive.setPower(WHEEL_SPEED_MULTIPLIER * rotationRight);
                narwhalHW.leftBackDrive.setPower(WHEEL_SPEED_MULTIPLIER * -rotationRight);
                narwhalHW.rightBackDrive.setPower(WHEEL_SPEED_MULTIPLIER * rotationRight);
            }
        }
        else {
            telemetry.addLine("LATERAL MODE");
            narwhalHW.leftFrontDrive.setPower(WHEEL_SPEED_MULTIPLIER * v1);
            narwhalHW.rightFrontDrive.setPower(WHEEL_SPEED_MULTIPLIER * v2);
            narwhalHW.leftBackDrive.setPower(WHEEL_SPEED_MULTIPLIER * v3);
            narwhalHW.rightBackDrive.setPower(WHEEL_SPEED_MULTIPLIER * v4);
        }

        narwhalHW.leftDrive.setPower(WHEEL_SPEED_MULTIPLIER * left);
        narwhalHW.rightDrive.setPower(WHEEL_SPEED_MULTIPLIER * right);

        telemetry.addData("leftDrive.Power", "%.2f", narwhalHW.leftDrive.getPower());
        telemetry.addData("rightDrive.Power", "%.2f", narwhalHW.rightDrive.getPower());
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    //@Override
    public void stop() {
    }
}
