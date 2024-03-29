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

@TeleOp(name="Narwhal: Driver Controlled Init", group="FreightFrenzy")
// @Disabled
public class PPE_NarwhalDriverControlledBase extends OpMode{
    /* Declare OpMode members. */
    public PPE_HardwareNarwhalChassis narwhalHWWheel;
    public PPE_HardwareNarwhalExternals2022 narwhalHWEx2022; // jjh Added 1/15/22

    boolean carouselIsMoving = false;
    boolean armSwivelIsMovingClockwise = false;
    boolean armSwivelIsMovingCounterClockwise = false;
    boolean armIsMovingUp = false;
    boolean armIsMovingDown = false;
    boolean flapperIsMoving = false;

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
         * The init() method of the hardware classes do all the work here
         */

        // use one of the classes created to define a STS_HardwareManatee's hardware
        narwhalHWWheel = new PPE_HardwareNarwhalMecanum(PPE_HardwareNarwhalChassis.driverMode.DRIVER_CONTROLLED, telemetry);
        //narwhalHWWheel = new PPE_HardwareNarwhalOmni(PPE_HardwareNarwhalChassis.driverMode.DRIVER_CONTROLLED, telemetry);
        narwhalHWWheel.init(hardwareMap);
        telemetry.addData("BREAKING:", "Narwhal Wheel Initialized");
        telemetry.update();
        sleep(1000);

        narwhalHWEx2022 = new PPE_HardwareNarwhalExternals2022(); // use the class created to define a STS_HardwareManatee's hardware
        narwhalHWEx2022.init(hardwareMap);
        telemetry.addData("BREAKING:", "Narwhal Externals Initialized");
        telemetry.update();
        sleep(1000);


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
        narwhalHWWheel.doLoop(gamepad1);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    //@Override
    public void stop() {
    }
}
