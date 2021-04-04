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
import com.qualcomm.robotcore.util.Range;


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

@TeleOp(name="Manatee: Driver Controlled", group="UltimateGoal")
// @Disabled
public class STS_ManateeDriverControlled extends OpMode{
    /* Declare OpMode members. */
    STS_HardwareManatee manatee = new STS_HardwareManatee(); // use the class created to define a Pushbot's hardware
    double              wobbleArmOffset = 0.0;
    double              wobbleClawOffset = 0.0;
    double              hopperOffset = 0.0;
    double              shooterAnglerOffset = 0.0;                         // Servo mid position

    boolean             shooterIsOn = false;
    boolean             intakeIsOn = false;
    boolean             armIsMovingForward = false;
    boolean             armIsMovingBackward = false;

    final double        WOBBLE_ARM_ANGLE = 0.001;                 // sets rate to move servo
    final double        WOBBLE_CLAW_ANGLE = 0.01;
    final double        HOPPER_ANGLE = 0.0001;
    final double        SHOOTER_ANGLER_ANGLE = 0.0002;

    final double        WHEEL_SPEED_MULTIPLIER = 1.0;
    final double        LATERAL_ADJUSTMENT      = 0.99;

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
        manatee.init(hardwareMap);

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
        double left = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;
        double turnRight = gamepad1.left_trigger;
        double turnLeft = gamepad1.right_trigger;
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
                manatee.leftFrontDrive.setPower(WHEEL_SPEED_MULTIPLIER * rotationLeft);
                manatee.rightFrontDrive.setPower(WHEEL_SPEED_MULTIPLIER * -rotationLeft);
                manatee.leftBackDrive.setPower(WHEEL_SPEED_MULTIPLIER * rotationLeft);
                manatee.rightBackDrive.setPower(WHEEL_SPEED_MULTIPLIER * -rotationLeft);
            }
            else {
                telemetry.addLine("ROTATION RIGHT MODE");
                manatee.leftFrontDrive.setPower(WHEEL_SPEED_MULTIPLIER * -rotationRight);
                manatee.rightFrontDrive.setPower(WHEEL_SPEED_MULTIPLIER * rotationRight);
                manatee.leftBackDrive.setPower(WHEEL_SPEED_MULTIPLIER * -rotationRight);
                manatee.rightBackDrive.setPower(WHEEL_SPEED_MULTIPLIER * rotationRight);
            }
        }
         else {
            telemetry.addLine("LATERAL MODE");
            manatee.leftFrontDrive.setPower(WHEEL_SPEED_MULTIPLIER * v1);
            manatee.rightFrontDrive.setPower(WHEEL_SPEED_MULTIPLIER * v2);
            manatee.leftBackDrive.setPower(WHEEL_SPEED_MULTIPLIER * v3);
            manatee.rightBackDrive.setPower(WHEEL_SPEED_MULTIPLIER * v4);
        }
/*
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        if ((turnLeft != 0) || (turnRight != 0)) {
            if (turnLeft != 0) {
                telemetry.addLine("TURN LEFT MODE");
                manatee.leftFrontDrive.setPower(WHEEL_SPEED_MULTIPLIER * turnLeft);
                manatee.rightFrontDrive.setPower(WHEEL_SPEED_MULTIPLIER * LATERAL_ADJUSTMENT * -turnLeft);
                manatee.leftBackDrive.setPower(WHEEL_SPEED_MULTIPLIER * turnLeft);
                manatee.rightBackDrive.setPower(WHEEL_SPEED_MULTIPLIER * LATERAL_ADJUSTMENT * -turnLeft);
            }
            else {
                telemetry.addLine("TURN RIGHT MODE");
                manatee.leftFrontDrive.setPower(WHEEL_SPEED_MULTIPLIER * -turnRight);
                manatee.rightFrontDrive.setPower(WHEEL_SPEED_MULTIPLIER * LATERAL_ADJUSTMENT * turnRight);
                manatee.leftBackDrive.setPower(WHEEL_SPEED_MULTIPLIER * -turnRight);
                manatee.rightBackDrive.setPower(WHEEL_SPEED_MULTIPLIER * LATERAL_ADJUSTMENT * turnRight);
            }
        }
        else {
            telemetry.addLine("LATERAL MODE");
            manatee.leftFrontDrive.setPower(WHEEL_SPEED_MULTIPLIER * left);
            manatee.rightFrontDrive.setPower(WHEEL_SPEED_MULTIPLIER * LATERAL_ADJUSTMENT * right);
            manatee.leftBackDrive.setPower(WHEEL_SPEED_MULTIPLIER * right);
            manatee.rightBackDrive.setPower(WHEEL_SPEED_MULTIPLIER * LATERAL_ADJUSTMENT * left);
        }

 */
        if (!STS_HardwareManatee.CHASSIS_ONLY) {
            if ((gamepad1.right_trigger > 0) && !shooterIsOn) {
                manatee.shooterWheelOne.setPower(1);
                manatee.shooterWheelTwo.setPower(-1);
                //wobbleArmOffset = 0;
                shooterIsOn = true;
            } else if ((gamepad1.right_trigger == 0) && shooterIsOn) {
                manatee.shooterWheelOne.setPower(0);
                manatee.shooterWheelTwo.setPower(0);
                shooterIsOn = false;
            }

            if ((gamepad1.left_trigger > 0) && !intakeIsOn) {
                manatee.intake.setPower(1);
                intakeIsOn = true;
            } else if ((gamepad1.left_trigger == 0) && intakeIsOn) {
                manatee.intake.setPower(0);
                intakeIsOn = false;
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
            */
            if (gamepad1.right_bumper) {
                manatee.hopper.setPosition(1.0);
            } else if (gamepad1.left_bumper) {
                manatee.hopper.setPosition(0.5);
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
        }
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
        if (!STS_HardwareManatee.CHASSIS_ONLY) {
            telemetry.addData("wobbleArm", "Offset = %.2f", wobbleArmOffset);
            telemetry.addData("wobbleClaw", "Offset = %.2f", wobbleClawOffset);
            telemetry.addData("hopper", "%.2f", manatee.hopper.getPosition());
            //telemetry.addData("shooterAngler", "Offset = %.2f", shooterAnglerOffset);
        }

        telemetry.addData("leftFrontDrive.Power", "%.2f", manatee.leftFrontDrive.getPower());
        telemetry.addData("rightFrontDrive.Power", "%.2f", manatee.rightFrontDrive.getPower());
        telemetry.addData("leftBackDrive.Power", "%.2f", manatee.leftBackDrive.getPower());
        telemetry.addData("rightBackDrive.Power", "%.2f", manatee.rightBackDrive.getPower());
        telemetry.addData("v1", "%.2f", v1);
        telemetry.addData("v2", "%.2f", v2);
        telemetry.addData("v3", "%.2f", v3);
        telemetry.addData("v4", "%.2f", v4);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
