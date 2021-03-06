/* Copyright (c) 2019 FIRST. All rights reserved.
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

import com.qualcomm.hardware.lynx.commands.standard.LynxSetModuleLEDPatternCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


import java.util.List;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Manatee: Autonomous", group = "UltimateGoal")
// @Disabled
public class STS_ManateeAutomomous extends STS_ManateeAutonomousInit {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final double SCAN_FOR_ELEMENT_TIMEOUT = 3.0;
    
    private ElapsedTime scanForElementTime = new ElapsedTime();
    private boolean foundElement = false;
    private boolean autonomousIsActive = true;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AdnLowb/////AAABmZNRoOYqak4+lc8AgsQ5vBMOIRDeD+0zoGNlSa//jZJWI6XfuWDNNrQY6PRX55edReDvHQXYkMdN+8nKqohfB8rn2AbVxq4LUeoe67LM4u5NVBGGS5teWMKQbXtqtRBgOkHRCgghllRNAfCOxKNdTT13e6fGNo8tgwQTwKiWHkNylKCBtlaS6ImDNRHQQ1Y1FBu7gr6qWlbvydsPZhnu1VDLMgUxNmK4HaYr/8Xm1865QBaPW9ePFPjHBuHm3h4k0M3Cj19QY2qbLJisNvH+uhkZF3PRxmGJiaeKxM8CkiDCj1JVvYhmjK/enFYGES7eVv4SYvmdizpJNtBrovFelExh25BKICZrdtWPkVm3zXDI";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    // @Override
    public void runOpMode() {
        super.runOpMode();

        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            tfod.setZoom(3.0, 1.78);
        }

        /** Commit all init information to the Driver Station */
        telemetry.update();

        /** Wait for the game to begin */
        waitForStart();

        while (opModeIsActive() && autonomousIsActive) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                scanForElementTime.reset();
                while (scanForElementTime.seconds() < SCAN_FOR_ELEMENT_TIMEOUT) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        // telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());

                            switch (recognition.getLabel()) {
                                case LABEL_FIRST_ELEMENT:
                                    telemetry.addLine("foundElement == LABEL_FIRST_ELEMENT");
                                    telemetry.update();
                                    sleep(250);
                                    encoderDrive(DriveMode.LINEAR, DRIVE_SPEED,  0,   100,  100, 10);
                                    if (!STS_HardwareManatee.CHASSIS_ONLY) {
                                        manatee.wobbleArm.setPosition(-0.1);
                                        sleep(250);
                                        manatee.wobbleClaw.setPosition(1.0);
                                        sleep(250);
                                        //encoderDrive(DriveMode.LINEAR, DRIVE_SPEED, 0, 5, 5, 10);
                                        manatee.wobbleArm.setPosition(1.0);
                                    }
                                    encoderDrive(DriveMode.LINEAR, DRIVE_SPEED,  0,   -20,  -20, 10);
                                    foundElement = true;
                                    autonomousIsActive = false;
                                    break;
                                case LABEL_SECOND_ELEMENT:
                                    telemetry.addLine("foundElement == LABEL_SECOND_ELEMENT");
                                    telemetry.update();
                                    sleep(250);
                                    encoderDrive(DriveMode.LINEAR, DRIVE_SPEED,  0,   98,  98, 10);
                                    encoderDrive(DriveMode.LINEAR, DRIVE_SPEED,  -90,   0,  0, 10);
                                    encoderDrive(DriveMode.LINEAR, DRIVE_SPEED,  0,   -44,  -44, 10);
                                    if (!STS_HardwareManatee.CHASSIS_ONLY) {
                                        manatee.wobbleArm.setPosition(-0.1);
                                        sleep(2000);
                                        manatee.wobbleClaw.setPosition(1.0);
                                        sleep(250);
                                        //encoderDrive(DriveMode.LINEAR, DRIVE_SPEED, 0, 5, 5, 10);
                                        manatee.wobbleArm.setPosition(1.0);
                                    }
                                    encoderDrive(DriveMode.LINEAR, DRIVE_SPEED,  0,   30,  30, 10);
                                    encoderDrive(DriveMode.LINEAR, DRIVE_SPEED,  90,   0,  0, 10);
                                    encoderDrive(DriveMode.LINEAR, DRIVE_SPEED,  0,   30,  30, 10);
                                    foundElement = true;
                                    autonomousIsActive = false;
                                    break;
                                default:
                                    // TODO: Is this possible or should I throw an
                                    //  UnsupportedOperationException
                                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                                    break;
                            }
                            tfod.shutdown();

                            telemetry.update();
                            // retInt = 1;
                        }
                    }  //if (updatedRecognitions != null)
                    else {
                        telemetry.addData("SCANNING FOR RINGS: time: ", scanForElementTime.seconds());
                        telemetry.update();
                    }
                }  // while (scanForElementTime.seconds() < SCAN_FOR_ELEMENT_TIMEOUT)

                if (!foundElement) {
                    // telemetry.addData("STS_ManateeAutonomousInit.DRIVE_SPEED == ", STS_ManateeAutonomousInit.DRIVE_SPEED);
                    telemetry.addLine("foundElement == LABEL_ZERO_ELEMENT");
                    telemetry.update();
                    sleep(250);
                    encoderDrive(DriveMode.LINEAR, DRIVE_SPEED,  0,   99,  99, 10);
                    encoderDrive(DriveMode.LINEAR, REAL_TURN_SPEED,  90,   0,  0, 10);
                    encoderDrive(DriveMode.LINEAR, DRIVE_SPEED,  0,   38,  38, 10);
                    if (!STS_HardwareManatee.CHASSIS_ONLY) {
                        manatee.wobbleArm.setPosition(-0.1);
                        sleep(2000);
                        manatee.wobbleClaw.setPosition(1.0);
                        sleep(500);
                        //encoderDrive(DriveMode.LINEAR, DRIVE_SPEED, 0, 3, 3, 10);
                        manatee.wobbleArm.setPosition(1.0);
                    }
                    encoderDrive(DriveMode.LINEAR, DRIVE_SPEED,  0,   -30,  -30, 10);
                    encoderDrive(DriveMode.LINEAR, REAL_TURN_SPEED,  90,   0,  0, 10);
                    encoderDrive(DriveMode.LINEAR, DRIVE_SPEED,  0,   26,  26, 10);
                    autonomousIsActive = false;
                }

                telemetry.addLine("Done with finding elements.");
                telemetry.update();
                sleep(2000);
            }
            else {
                telemetry.addData("tfod != null", "ERROR!!!");
                telemetry.update();
                sleep(5000);
            } // if (tfod != null)

            if (tfod != null) {
                tfod.shutdown();
            }
        } // while (opModeIsActive())
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        telemetry.addLine(parameters.cameraName.toString());

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
