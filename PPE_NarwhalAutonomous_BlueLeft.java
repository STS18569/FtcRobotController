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

@Autonomous(name="Narwhal: Blue Left Autonomous", group="UFreightFrenzy")
//@Disabled
public class PPE_NarwhalAutonomous_BlueLeft extends PPE_NarwhalAutonomousBase {

    @Override
    public void runAutonomousMode() {
        super.runOpMode();


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
                    String objectLocation = "";
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

                            //Set of Conditionals that identifies the location of the starting element
                            if (recognition.getLeft() > 5 && recognition.getLeft() < 132){
                                objectLocation = "left";
                            } else if(recognition.getLeft() > 224 && recognition.getLeft() < 360){
                                objectLocation = "middle";
                            } else if(recognition.getLeft() > 440 && recognition.getLeft() < 523){
                                objectLocation = "right";
                            }

                            telemetry.addLine(objectLocation);
                            telemetry.update();

                            if (recognition.getLabel() == "TempSE")
                                break;

                            // retInt = 1;
                        }
                    }  //if (updatedRecognitions != null)
                    else {
                        telemetry.addData("SCANNING FOR Shipping Element: time: ", scanForElementTime.seconds());
                        telemetry.update();
                    }

                    if (objectLocation != "")
                        break;

                }  // while (scanForElementTime.seconds() < SCAN_FOR_ELEMENT_TIMEOUT)



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




        initVuforia();
        initTfod();



    }

}
