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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

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


@TeleOp(name="Narwhal: Driver Controlled", group="FreightFrenzy")
// @Disabled
public class PPE_NarwhalDriverControlled extends PPE_NarwhalDriverControlledBase {
    private ElapsedTime runtime = new ElapsedTime();

    /*
    Insert useMap1.a = runtime.milliseconds(); after every use of gamepad1.a to reset the cooldown.
    Use toggleMap1.a to access whether gamepad1.a is toggled or not.
    cdCheck(useMap1.a, 1000) returns true or false based on whether gamepad1.a has been used in the last 1000 milliseconds.
    */

    toggleMap toggleMap1 = new toggleMap();
    useMap useMap1 = new useMap();

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

        super.loop();

        updateKeys();

        if (gamepad2.left_bumper && !armSwivelIsMovingCounterClockwise) {
            narwhalHWEx2022.armSwivel.setPower(-0.4);
            armSwivelIsMovingCounterClockwise = true;
        } else if (!gamepad2.left_bumper && armSwivelIsMovingCounterClockwise) {
            narwhalHWEx2022.armSwivel.setPower(0);
            armSwivelIsMovingCounterClockwise = false;
        }

        if (gamepad2.right_bumper && !armSwivelIsMovingClockwise) {
            narwhalHWEx2022.armSwivel.setPower(0.4);
            armSwivelIsMovingClockwise = true;
        } else if (!gamepad2.right_bumper && armSwivelIsMovingClockwise) {
            narwhalHWEx2022.armSwivel.setPower(0);
            armSwivelIsMovingClockwise = false;
        }

        if ((gamepad2.left_trigger > 0) && !armIsMovingUp) {
            narwhalHWEx2022.arm.setPower(0.4);
            armIsMovingUp = true;
        } else if (!(gamepad2.left_trigger > 0) && armIsMovingUp) {
            narwhalHWEx2022.arm.setPower(0);
            armIsMovingUp = false;
        }

        if ((gamepad2.right_trigger > 0) && !armIsMovingDown) {
            narwhalHWEx2022.arm.setPower(-0.25);
            armIsMovingDown = true;
        } else if (!(gamepad2.right_trigger > 0) && armIsMovingDown) {
            narwhalHWEx2022.arm.setPower(0);
            armIsMovingDown = false;
        }

/*
        if (gamepad1.dpad_down) {
            narwhalHWEx2022.angularArmDrive(STS_HardwareBotBoy.ArmPosition.REST, 0.4, 2.0);
        }
        if (gamepad1.dpad_left) {
            narwhalHWEx2022.angularArmDrive(STS_HardwareBotBoy.ArmPosition.MIDDLE, 0.4, 2.0);
        }
        if (gamepad1.dpad_up) {
            narwhalHWEx2022.angularArmDrive(STS_HardwareBotBoy.ArmPosition.TOP, 0.4, 2.0);
        }
        if (gamepad1.dpad_right) {
            narwhalHWEx2022.angularArmDrive(STS_HardwareBotBoy.ArmPosition.BOTTOM, 0.4, 2.0);
        }
        if ((gamepad1.left_stick_y != 0 || gamepad1.right_stick_y != 0) && (narwhalHWEx2022.arm.getCurrentPosition() == 0)) {
            narwhalHWEx2022.angularArmDrive(STS_HardwareBotBoy.ArmPosition.RAISED, 0.4, 2.0);
        }
 */

        // Send telemetry message to signify robot running;
        telemetry.addData("cross (flapper)", toggleMap1.cross + " " + (runtime.milliseconds() - useMap1.cross));
        telemetry.addData("circle (carousel)", toggleMap1.circle + " " + (runtime.milliseconds() - useMap1.circle));
        telemetry.addData("square (intake)", toggleMap1.square + " " + (runtime.milliseconds() - useMap1.square));
        telemetry.addData("swivel position",  "Running at %7d", narwhalHWEx2022.armSwivel.getCurrentPosition());
        telemetry.addData("arm position",  "Running at %7d", narwhalHWEx2022.arm.getCurrentPosition());
        telemetry.update();
    }

    public void updateKeys() {      // This section is for buttons that use toggle rather than hold
        if(gamepad2.circle && cdCheck(useMap1.circle, 500) && !carouselIsMoving){
            toggleMap1.circle = toggle(toggleMap1.circle);
            useMap1.circle = runtime.milliseconds();
            carouselIsMoving = true;
            narwhalHWEx2022.carousel.setPower(0.75);
        }

        if(gamepad2.circle && cdCheck(useMap1.circle, 500) && carouselIsMoving){
            toggleMap1.circle = toggle(toggleMap1.circle);
            useMap1.circle = runtime.milliseconds();
            carouselIsMoving = false;
            narwhalHWEx2022.carousel.setPower(0.0);
        }

        if(gamepad2.square && cdCheck(useMap1.square, 500) && !carouselIsMoving){
            toggleMap1.square = toggle(toggleMap1.square);
            useMap1.square = runtime.milliseconds();
            carouselIsMoving = true;
            narwhalHWEx2022.carousel.setPower(-0.75);
        }

        if(gamepad2.square && cdCheck(useMap1.square, 500) && carouselIsMoving){
            toggleMap1.square = toggle(toggleMap1.square);
            useMap1.square = runtime.milliseconds();
            carouselIsMoving = false;
            narwhalHWEx2022.carousel.setPower(0.0);
        }

        if(gamepad2.triangle && cdCheck(useMap1.triangle, 500) && !flapperIsMoving){
            toggleMap1.triangle = toggle(toggleMap1.triangle);
            useMap1.triangle = runtime.milliseconds();
            flapperIsMoving = true;
            narwhalHWEx2022.flapper.setPower(1.0);
        }

        if(gamepad2.triangle && cdCheck(useMap1.triangle, 500) && flapperIsMoving){
            toggleMap1.triangle = toggle(toggleMap1.triangle);
            useMap1.triangle = runtime.milliseconds();
            flapperIsMoving = false;
            narwhalHWEx2022.flapper.setPower(0.0);
        }

        if(gamepad2.cross && cdCheck(useMap1.cross, 500) && !flapperIsMoving){
            toggleMap1.cross = toggle(toggleMap1.cross);
            useMap1.cross = runtime.milliseconds();
            flapperIsMoving = true;
            narwhalHWEx2022.flapper.setPower(-1.0);
        }

        if(gamepad2.cross && cdCheck(useMap1.cross, 500) && flapperIsMoving){
            toggleMap1.cross = toggle(toggleMap1.cross);
            useMap1.cross = runtime.milliseconds();
            flapperIsMoving = false;
            narwhalHWEx2022.flapper.setPower(0.0);
        }
        /*
        if(gamepad1.right_stick_x > 0 && cdCheck(useMap1.right_stick_x_pos, 700)){
            toggleMap1.right_stick_x_pos = toggle(toggleMap1.right_stick_x_pos);
            useMap1.right_stick_x_pos = runtime.milliseconds();
        }

         */
    }

    public boolean cdCheck(double key, int cdTime){
        return runtime.milliseconds() - key > cdTime;
    }

    public boolean toggle(boolean variable){
        if(variable == true){
            variable = false;
        }
        else if(variable == false){
            variable = true;
        }
        return variable;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    //@Override
    public void stop() {
    }
}
