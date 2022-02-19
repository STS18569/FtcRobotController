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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is used as a base class for a Decorator implementation.
 */
public class PPE_HardwareNarwhalChassis
{
    protected static final double WHEEL_SPEED_MULTIPLIER = 0.5;
    protected static final double WHEEL_TURN_SPEED_MULTIPLIER = 0.9;
    protected static final double WHEEL_BASE = 10.8;
    protected static final double COUNTS_PER_MOTOR_REV = 28.0;    // eg: TETRIX Motor Encoder
    protected static final double DRIVE_GEAR_REDUCTION = 20.0;     // This is < 1.0 if geared UP
    protected static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    protected static final double FUDGE_FACTOR = 1;
    protected static final double COUNTS_PER_INCH = 40; //FUDGE_FACTOR * ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415));

    protected static enum driverMode {AUTONOMOUS, DRIVER_CONTROLLED};
    protected static enum DriveMode {LAT_LEFT, LAT_RIGHT, LINEAR};

    protected HardwareMap hwMap = null;
    protected driverMode hwDriverMode = null;
    protected Telemetry telemetry = null;

    protected ElapsedTime period  = new ElapsedTime();
    protected ElapsedTime runtime = new ElapsedTime();

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException var4) {
            Thread.currentThread().interrupt();
        }
    }

    public PPE_HardwareNarwhalChassis(driverMode hwDriverMode, Telemetry telemetry)
    {
        this.hwDriverMode = hwDriverMode;
        this.telemetry = telemetry;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap)
    {
        hwMap = ahwMap;
    }

    public void doLoop(Gamepad gamepad1)
    {
        // Do nothing (essentially an abstract method)
    }

    public void encoderDrive(double speed, double degree,
                             double leftInches, double rightInches,
                             double timeoutS, DriveMode mode, LinearOpMode curLinearOpMode)
    {
        // Do nothing (essentially an abstract method)
    }

    //This eventually can be removed; for testing right now
    public void encoderDriveTest1(double speed, double degree,
                                  double leftInches, double rightInches,
                                  double timeoutS, PPE_HardwareNarwhalChassis.DriveMode mode, LinearOpMode curLinearOpMode) {
        // Do nothing (essentially an abstract method)
    }
}

