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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;


@Autonomous(name="Narwhal: Blue Right Autonomous", group="FreightFrenzy")
//@Disabled
public class PPE_NarwhalAutonomous_BlueRight extends PPE_NarwhalAutonomousBase {

    @Override
    public void runAutonomousMode() {

        //Base Auto
        //Carousel(10) + Park (3 ~ 6)
        narwhalHWWheel.encoderDrive(DRIVE_SPEED,0,7,  7, 2.0, PPE_HardwareNarwhalChassis.DriveMode.LAT_RIGHT,this);
        narwhalHWWheel.encoderDrive(DRIVE_SPEED, 0, -30, -30, 3.0,PPE_HardwareNarwhalChassis.DriveMode.LINEAR, this );
        narwhalHWEx2022.carousel.setPower(0.6);
        sleep(5000);
        narwhalHWEx2022.carousel.setPower(0);
        narwhalHWWheel.encoderDrive(DRIVE_SPEED,0,28,  28, 3.0, PPE_HardwareNarwhalChassis.DriveMode.LAT_RIGHT,this);
        narwhalHWWheel.encoderDrive(DRIVE_SPEED, 0, -5, -5, 3.0,PPE_HardwareNarwhalChassis.DriveMode.LINEAR, this );


        /*
        TODO: While Vision was tested and was working to be able to identify randomization, mechanical limitations prevented this from being used at all
        initVuforia();
        initTfod();
        String objectLocation = "";




        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
            tfod.setClippingMargins(1500,500,1500,500);
        }

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && autonomousIsActive) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                scanForElementTime.reset();

                while (scanForElementTime.seconds() < SCAN_FOR_ELEMENT_TIMEOUT) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;

                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            telemetry.addData(String.format("  confidence (%d)", i), "%.03f",
                                    recognition.getConfidence());
                            i++;

                            //Set of Conditionals that identifies the location of the starting element
                            if (recognition.getLeft() > 5 && recognition.getLeft() < 132){
                                objectLocation = "left";
                                break;
                            } else if(recognition.getLeft() > 224 && recognition.getLeft() < 360){
                                objectLocation = "middle";
                                break;
                            } else if(recognition.getLeft() > 440 && recognition.getLeft() < 523){
                                objectLocation = "right";
                                break;
                            }
                            telemetry.addLine(objectLocation);
                        }
                        telemetry.update();

                        //Left range: (5, 93) (132, 212)
                        //Middle range:(224, 306), (360, 442)
                        //Right range: (440, 526), (523, 618)


                    }  //if (updatedRecognitions != null)
                    else {
                        telemetry.addData("SCANNING FOR RINGS: time: ", scanForElementTime.seconds());
                        telemetry.update();
                    }

                    if (objectLocation != ""){
                        break;
                    }
                }  // while (scanForElementTime.seconds() < SCAN_FOR_ELEMENT_TIMEOUT)

                if (!foundElement) {
                    // telemetry.addData("STS_ManateeAutonomousInit.DRIVE_SPEED == ", STS_ManateeAutonomousInit.DRIVE_SPEED);
                    telemetry.addLine("foundElement == LABEL_ZERO_ELEMENT");
                    telemetry.update();
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
        }

        telemetry.addLine(objectLocation);
        telemetry.update();

        switch (objectLocation){
            case "left":
                narwhalHWWheel.encoderDrive(DRIVE_SPEED, 0, 10, 10, 3.0,PPE_HardwareNarwhalChassis.DriveMode.LAT_LEFT, this );
                break;
            case "middle":
                narwhalHWWheel.encoderDrive(DRIVE_SPEED, 0, 10, 10, 3.0,PPE_HardwareNarwhalChassis.DriveMode.LINEAR, this );
                break;
            case "right":
                narwhalHWWheel.encoderDrive(DRIVE_SPEED, 0, 10, 10, 3.0,PPE_HardwareNarwhalChassis.DriveMode.LAT_RIGHT, this );
                break;
            default:

        */
    }
}
