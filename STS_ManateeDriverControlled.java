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

@TeleOp(name="Manatee: Driver Controlled", group="Freight Frenzy")
// @Disabled
public class STS_ManateeDriverControlled extends STS_ManateeDriverControlledInit {

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        super.init();
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
        if (gamepad1.left_bumper && !armIsOn) {
            manatee.arm.setPower(0.2);
            armIsOn = true;
        } else if (!gamepad1.left_bumper && armIsOn) {
            manatee.arm.setPower(0);
            armIsOn = false;
        }

        if (gamepad1.right_bumper && !armIsMovingBackward) {
            manatee.arm.setPower(-0.2);
            armIsMovingBackward = true;
        } else if (!gamepad1.right_bumper && armIsMovingBackward) {
            manatee.arm.setPower(0);
            armIsMovingBackward = false;
        }

        if (gamepad1.square) {
            intakeLeftOffset += INTAKE_ANGLE;
            intakeRightOffset -= INTAKE_ANGLE;
        }

        /*
        if (gamepad1.y && !armIsMovingForward) {
            manatee.wobbleArm.setPower(0.5);
            armIsMovingForward = true;
        } else if (!gamepad1.y && armIsMovingForward) {
            manatee.wobbleArm.setPower(0);
            armIsMovingForward = false;
        }

        if (gamepad1.a && !armIsMovingBackward) {
            manatee.wobbleArm.setPower(-0.5);
            armIsMovingBackward = true;
        } else if (!gamepad1.a && armIsMovingBackward) {
            manatee.wobbleArm.setPower(0);
            armIsMovingBackward = false;
        }

        if (gamepad1.dpad_up) {
            shooterAnglerOffset += SHOOTER_ANGLER_ANGLE;
        } else if (gamepad1.dpad_down) {
            shooterAnglerOffset -= SHOOTER_ANGLER_ANGLE;
        }

        if (gamepad1.right_bumper) {
            manatee.hopper.setPosition(1.0);
        } else if (gamepad1.left_bumper) {
            manatee.hopper.setPosition(0.45);
        }

        if (gamepad1.a) {
            wobbleArmOffset += WOBBLE_ARM_ANGLE;
        } else if (gamepad1.y) {
            wobbleArmOffset -= WOBBLE_ARM_ANGLE;
        }

        if (gamepad1.x) {
            wobbleClawOffset += WOBBLE_CLAW_ANGLE;
        } else if (gamepad1.b) {
            wobbleClawOffset -= WOBBLE_CLAW_ANGLE;
        }

        // Move both servos to new position.  Assume servos are mirror image of each other.
        wobbleArmOffset = Range.clip(wobbleArmOffset, 0, 2);
        wobbleClawOffset = Range.clip(wobbleClawOffset, -1, 1);
        hopperOffset = Range.clip(hopperOffset, -1, 1);
        //shooterAnglerOffset = Range.clip(shooterAnglerOffset, 0.15, 0.2);

        manatee.wobbleArm.setPosition(manatee.WOBBLE_ARM_MID_SERVO + wobbleArmOffset);
        manatee.wobbleClaw.setPosition(manatee.WOBBLE_CLAW_MID_SERVO + wobbleClawOffset);
        //manatee.shooterAngler.setPosition(manatee.SHOOTER_ANGLER_MID_SERVO + shooterAnglerOffset);
        */

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
        telemetry.addData("intakeLeft", "%.2f", manatee.intakeLeft);
        telemetry.addData("intakeRight", "%.2f", manatee.intakeRight);

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    //@Override
    public void stop() {
    }
}
