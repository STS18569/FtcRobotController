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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

import static java.lang.Thread.sleep;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Driver Controlled Mode", group="Pushbot")
// @Disabled
public class STS_PushbotTeleopTank_Iterative extends OpMode{

    /* Declare OpMode members. */
    STS_HardwarePushbot robot       = new STS_HardwarePushbot(); // use the class created to define a Pushbot's hardware
    double          wobbleArmOffset  = 0.0;
    double          wobbleClawOffset  = 0.0;
    double          shooterAnglerOffset  = 0.0;                         // Servo mid position
    final double    WOBBLE_ARM_ANGLE  = 0.02;                 // sets rate to move servo
    final double    WOBBLE_CLAW_ANGLE  = 0.02;
    final double    SHOOTER_ANGLER_ANGLE  = 0.02;
    final double    WHEEL_SPEED_MULTIPLYER = 0.8;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
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
        double left;
        double right;
        double turnleft;
        double turnright;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;

        robot.leftFrontDrive.setPower(WHEEL_SPEED_MULTIPLYER*left);
        robot.rightFrontDrive.setPower(WHEEL_SPEED_MULTIPLYER*right);
        robot.leftBackDrive.setPower(WHEEL_SPEED_MULTIPLYER*right);
        robot.rightBackDrive.setPower(WHEEL_SPEED_MULTIPLYER*left);

        turnright = gamepad1.left_trigger;
        turnleft = gamepad1.right_trigger;

        robot.leftFrontDrive.setPower(turnleft);
        robot.rightFrontDrive.setPower(turnright);
        robot.leftBackDrive.setPower(turnleft);
        robot.rightBackDrive.setPower(turnright);


        if (gamepad1.right_bumper) {
            robot.shooterWheel.setPower(1);
            try {
                sleep(10000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            robot.shooterWheel.setPower(0);
        }

        if (gamepad1.left_bumper) {
            robot.intakeWheel.setPower(1);
            try {
                sleep(10000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            robot.intakeWheel.setPower(0);
        }

        if (gamepad1.y) {
            shooterAnglerOffset += SHOOTER_ANGLER_ANGLE;
        }
        else if (gamepad1.a) {
            shooterAnglerOffset -= SHOOTER_ANGLER_ANGLE;
        }

        if (gamepad1.dpad_up) {
            wobbleArmOffset += WOBBLE_ARM_ANGLE;
        }
        else if (gamepad1.dpad_down) {
            wobbleArmOffset -= WOBBLE_ARM_ANGLE;
        }

        if (gamepad1.dpad_right) {
            wobbleClawOffset += WOBBLE_CLAW_ANGLE;
        }
        else if (gamepad1.dpad_left) {
            wobbleClawOffset -= WOBBLE_CLAW_ANGLE;
        }

        // Move both servos to new position.  Assume servos are mirror image of each other.
        wobbleArmOffset = Range.clip(wobbleArmOffset, -0.5, 0.5);
        wobbleClawOffset = Range.clip(wobbleClawOffset, -0.5, 0.5);
        shooterAnglerOffset = Range.clip(shooterAnglerOffset, -0.5, 0.5);
        robot.wobbleArm.setPosition(robot.MID_SERVO + wobbleArmOffset);
        robot.wobbleClaw.setPosition(robot.MID_SERVO + wobbleClawOffset);
        robot.shooterAngler.setPosition(robot.MID_SERVO + wobbleArmOffset);

        /*
        // Use gamepad buttons to move the arm up (Y) and down (A)
        if (gamepad1.y)
            robot.leftArm.setPower(robot.ARM_UP_POWER);
        else if (gamepad1.a)
            robot.leftArm.setPower(robot.ARM_DOWN_POWER);
        else
            robot.leftArm.setPower(0.0);
*/

        // Send telemetry message to signify robot running;
        telemetry.addData("wobbleArm",  "Offset = %.2f", wobbleArmOffset);
        telemetry.addData("wobbleClaw",  "Offset = %.2f", wobbleClawOffset);
        telemetry.addData("shooterAngler",  "Offset = %.2f", shooterAnglerOffset);
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
