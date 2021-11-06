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

@TeleOp(name="Manatee: Driver Controlled Init", group="`Freight Frenzy`")
// @Disabled
public class STS_BotBoyDriverControlledInit extends OpMode{
    /* Declare OpMode members. */
    STS_HardwareBotBoy manatee = new STS_HardwareBotBoy(); // use the class created to define a STS_HardwareManatee's hardware


    double              armLidOffset      = 0.0;
    double              intakeLeftOffset  = 0.0;
    double              intakeRightOffset = 0.0;

    boolean             armIsMovingForward = false;
    boolean             armIsMovingBackward = false;

    double              ARM_LID_SPEED = 0.1;
    double              INTAKE_SPEED = 0.2;

    final double        WHEEL_SPEED_MULTIPLIER = 0.5;
    // final double        LATERAL_ADJUSTMENT     = 0.99;

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
        double left = gamepad1.left_stick_y;
        double right = gamepad1.right_stick_y;
        double middleRight = gamepad1.right_trigger;
        double middleLeft = gamepad1.left_trigger;
        double rotationLeft = gamepad1.right_stick_x;
        double rotationRight = -gamepad1.right_stick_x;

        manatee.leftDrive.setPower(WHEEL_SPEED_MULTIPLIER * left);
        manatee.rightDrive.setPower(WHEEL_SPEED_MULTIPLIER * right);
        manatee.middleDrive.setPower(WHEEL_SPEED_MULTIPLIER * middleLeft);
        manatee.middleDrive.setPower(-WHEEL_SPEED_MULTIPLIER * middleRight);

        telemetry.addData("leftDrive.Power", "%.2f", manatee.leftDrive.getPower());
        telemetry.addData("rightDrive.Power", "%.2f", manatee.rightDrive.getPower());
        telemetry.addData("middleDrive.Power", "%.2f", manatee.middleDrive.getPower());

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    //@Override
    public void stop() {
    }
}
