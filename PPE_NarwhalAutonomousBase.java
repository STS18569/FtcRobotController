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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.*;

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
public abstract class PPE_NarwhalAutonomousBase extends LinearOpMode
{
    /* Declare OpMode members. */
    protected PPE_HardwareNarwhalChassis narwhalHWWheel;
    protected PPE_HardwareNarwhalExternals2022 narwhalHWEx2022; // jjh Added 1/15/22

    static enum DriveMode {LAT_LEFT, LAT_RIGHT, LINEAR};

    static final double COUNTS_PER_MOTOR_REV = 28.0;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 4.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.5;     // For figuring circumference
    static final double FUDGE_FACTOR = 0.89;
    static final double DRIVE_SPEED = 0.5;
    static final double LATERAL_ADJUSTMENT = 1.0;
    static final double TURN_SPEED = 0.8;
    static final double REAL_TURN_SPEED = 0.35;

    static final String TFOD_MODEL_ASSET = "STS_Model.tflite";
    static final String[] LABELS = {
            "Box",
            "Ball",
            "Duck",
            "TempSE"
    };

    static final String VUFORIA_KEY =
            "AdnLowb/////AAABmZNRoOYqak4+lc8AgsQ5vBMOIRDeD+0zoGNlSa//jZJWI6XfuWDNNrQY6PRX55edReDvHQXYkMdN+8nKqohfB8rn2AbVxq4LUeoe67LM4u5NVBGGS5teWMKQbXtqtRBgOkHRCgghllRNAfCOxKNdTT13e6fGNo8tgwQTwKiWHkNylKCBtlaS6ImDNRHQQ1Y1FBu7gr6qWlbvydsPZhnu1VDLMgUxNmK4HaYr/8Xm1865QBaPW9ePFPjHBuHm3h4k0M3Cj19QY2qbLJisNvH+uhkZF3PRxmGJiaeKxM8CkiDCj1JVvYhmjK/enFYGES7eVv4SYvmdizpJNtBrovFelExh25BKICZrdtWPkVm3zXDI";
    protected VuforiaLocalizer vuforia;
    protected TFObjectDetector tfod;


    static final double SCAN_FOR_ELEMENT_TIMEOUT = 3.0;

    protected ElapsedTime scanForElementTime = new ElapsedTime();
    protected boolean foundElement = false;
    protected boolean autonomousIsActive = true;

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        narwhalHWWheel = new PPE_HardwareNarwhalMecanum(PPE_HardwareNarwhalChassis.driverMode.AUTONOMOUS, telemetry);
        // narwhalHWWheel = new PPE_HardwareNarwhalOmni(PPE_HardwareNarwhalWheel.driverMode.AUTONOMOUS, telemetry);

        narwhalHWWheel.init(hardwareMap);

        narwhalHWEx2022 = new PPE_HardwareNarwhalExternals2022(); // use the class created to define a STS_HardwareManatee's hardware
        narwhalHWEx2022.init(hardwareMap);

        // Send telemetry message to indicate successful Encoder reset

        waitForStart();

        runAutonomousMode();
    }

    public abstract void runAutonomousMode();

    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }







    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

    //TODO: ENCODERDRIVE HIGHLY UNRELIABLE, OFTEN OVERSHOT OR UNDERSHOT TURN
    /*
    public void encoderDriveOmni(double speed, double degree,
                                 double leftInches, double rightInches,
                                 double timeoutS, LinearOpMode curLinearOpMode)

    public void encoderDriveMecanum(double speed, double degree,
                                    double leftInches, double rightInches,
                                    double timeoutS, LinearOpMode curLinearOpMode)
     */
}
