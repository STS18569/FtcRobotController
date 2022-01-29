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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class PPE_HardwareNarwhalOmni extends PPE_HardwareNarwhalChassis
{
    /* local OpMode members. */
    public DcMotor  leftDrive           = null;
    public DcMotor  rightDrive          = null;

    /* Constructor */
    public PPE_HardwareNarwhalOmni(PPE_HardwareNarwhalChassis.driverMode hwDriverMode, Telemetry telemetry) {
        super(hwDriverMode, telemetry);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);

        // Define and Initialize Motors
        leftDrive   = hwMap.get(DcMotor.class, "left_drive");
        rightDrive  = hwMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        //Set zero power behavior
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0", "Starting at %7d :%7d",
                leftDrive.getCurrentPosition(),
                rightDrive.getCurrentPosition());
    }

    public void doLoop(Gamepad gamepad1)
    {
        double left = gamepad1.left_stick_y;
        double right = gamepad1.right_stick_y;

        leftDrive.setPower(WHEEL_SPEED_MULTIPLIER * left);
        rightDrive.setPower(WHEEL_SPEED_MULTIPLIER * right);

        telemetry.addData("leftDrive.Power", "%.2f", leftDrive.getPower());
        telemetry.addData("rightDrive.Power", "%.2f", rightDrive.getPower());
    }

    public void encoderDrive(double speed, double degree,
                             double leftInches, double rightInches,
                             double timeoutS, PPE_HardwareNarwhalChassis.DriveMode mode, LinearOpMode curLinearOpMode) {
        super.encoderDrive(speed, degree, leftInches, rightInches, timeoutS, mode, curLinearOpMode);

        if (degree < 0) {
            leftInches = -(degree * ((WHEEL_BASE * Math.PI) / 360));
            rightInches = ((degree * ((WHEEL_BASE * Math.PI) / 360)));
        } else if (degree > 0) {
            leftInches = -((degree * ((WHEEL_BASE * Math.PI) / 360)));
            rightInches = degree * ((WHEEL_BASE * Math.PI) / 360);
        } else if (degree == 0) {
            leftInches = leftInches;
            rightInches = rightInches;
        }

        int newLeftTarget = 0;
        int newRightTarget = 0;

        // Ensure that the opmode is still active
        if (curLinearOpMode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller

            newLeftTarget = leftDrive.getCurrentPosition() + (int) (-leftInches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int) (-rightInches * COUNTS_PER_INCH);
            // code block

            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            // telemetry.addData("speed == ", speed);
            // telemetry.addData("timeoutS == ", timeoutS);
            while (curLinearOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    ((leftDrive.isBusy() || rightDrive.isBusy()))) {

                // Display it for the driver.
                /*
                telemetry.addData("speed",  speed);
                telemetry.addData("degree",  degree);
                telemetry.addData("leftInches",  leftInches);
                telemetry.addData("rightInches",  rightInches);
                telemetry.addData("timeoutS",  timeoutS);
                 */
                telemetry.addData("Path1 (target)", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2 (position)", "Running at %7d :%7d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                // telemetry.addData("EncoderDrive", "time(%3d) : %3f", timeoutS, runtime.seconds());
                telemetry.addData("EncoderDrive: time: ", runtime.seconds());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    //TODO: ENCODERDRIVES NEED TO BE FINSIHED AND NEED TO BE REMADE FOR NEW FILE

    public void encoderDriveTest1(double speed, double degree,
                                      double leftInches, double rightInches,
                                      double timeoutS, PPE_HardwareNarwhalChassis.DriveMode mode, LinearOpMode curLinearOpMode) {
        super.encoderDrive(speed, degree, leftInches, rightInches, timeoutS, mode, curLinearOpMode);
        // Declares variables that are used for this method

        if (degree < 0) {
            leftInches = degree*((WHEEL_BASE * Math.PI)/360);
            rightInches = -((degree*((WHEEL_BASE * Math.PI)/360)));
        }

        else if (degree > 0) {
            rightInches = degree*((WHEEL_BASE * Math.PI)/360);
            leftInches = -((degree*((WHEEL_BASE * Math.PI)/360)));
        }

        else if (degree == 0) {
            leftInches = leftInches;
            rightInches = rightInches;
        }

        int NewLeftTarget;
        int NewRightTarget;
        int RightPosition;
        int LeftPosition;
        double LeftPower;
        double RightPower;

        // Resets encoders to 0
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        if (curLinearOpMode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            // Calculates the needed encoder ticks by multiplying a pre-determined amount of CountsPerInches,
            // and the method input gets the actual distance travel in inches
            NewLeftTarget = leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            NewRightTarget = rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            // Gets the current position of the encoders at the beginning of the EncoderDrive method
            RightPosition = rightDrive.getCurrentPosition();
            LeftPosition = leftDrive.getCurrentPosition();

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(speed);
            rightDrive.setPower(speed);
            boolean Running = true;
            /*
            double LeftEncoderPositionAtFullSpeed;
            double RightEncoderPositionAtFullSpeed;
            boolean Running;
            // This gets where the motor encoders will be at full position when it will be at full speed.
            if ((leftInches > 3) && (rightInches > 3)) {
                LeftEncoderPositionAtFullSpeed = ((3 * (COUNTS_PER_INCH)) + LeftPosition);
                RightEncoderPositionAtFullSpeed = ((3 * (COUNTS_PER_INCH)) + RightPosition);
                Running = true;
            } else {
                LeftEncoderPositionAtFullSpeed = (LeftPosition);
                RightEncoderPositionAtFullSpeed = (RightPosition);
                Running = true;
            }

             */

            // This gets the absolute value of the encoder positions at full speed - the current speed, and while it's greater than 0, it will continues increasing the speed.
            // This allows the robot to accelerate over a set number of inches, which reduces wheel slippage and increases overall reliability
            while (leftDrive.isBusy() && rightDrive.isBusy()
                    && Running && curLinearOpMode.opModeIsActive() && (runtime.seconds() < timeoutS)) {
                //
                if (((leftDrive.getTargetPosition() - leftDrive.getCurrentPosition()) <= (4 * COUNTS_PER_INCH)) && ((Math.abs(leftDrive.getPower()) > .05))) {
                    // This allows the robot to accelerate over a set distance, rather than going full speed.  This reduces wheel slippage and increases reliability.
                    LeftPower = (Range.clip(Math.abs((leftDrive.getPower()) * 0.001), .15, speed));
                    RightPower = (Range.clip(Math.abs((rightDrive.getPower()) * 0.001), .15, speed));

                    leftDrive.setPower(LeftPower);
                    rightDrive.setPower(RightPower);

                    //telemetry.addData("In Accel loop CM", + Distance.getDistance(DistanceUnit.CM));
                    //telemetry.addData("Accelerating", RightEncoderPositionAtFullSpeed);
                    telemetry.addData("Path1", "Running to %7d :%7d", NewLeftTarget, NewRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                    //telemetry.addData("Sections Complete:", +SectionsCompleted);
                    telemetry.update();
                    //This if statement might be kinda pointless... leaving it in for now
                } else if (((Math.abs(NewLeftTarget) - Math.abs(leftDrive.getCurrentPosition()) < COUNTS_PER_INCH) || (Math.abs(NewLeftTarget) - Math.abs(leftDrive.getCurrentPosition()) > COUNTS_PER_INCH))
                        && ((Math.abs(NewRightTarget) - Math.abs(rightDrive.getCurrentPosition()) < COUNTS_PER_INCH) || (Math.abs(NewRightTarget) - Math.abs(rightDrive.getCurrentPosition()) > COUNTS_PER_INCH))) {
                    Running = false;
                }

                 else {

                    // Multiplies the speed desired by the direction, which has a value of either 1, or -1, and allows for backwards driving with the ramp up function
                    leftDrive.setPower(speed);
                    rightDrive.setPower(speed);

                    //telemetry.addData("In Reg loop CM", + Distance.getDistance(DistanceUnit.CM));
                    telemetry.addData("Path1", "Running to %7d :%7d", NewLeftTarget, NewRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                    //telemetry.addData("Sections Complete:", +SectionsCompleted);
                    telemetry.update();


                }


                // Display information for the driver.

            }

            // Stops all motion
            // Set to run without encoder, so it's not necessary to declare this every time after the method is used
            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Set power to 0
            leftDrive.setPower(0);
            rightDrive.setPower(0);


        }
    }
/*

    public void encoderDriveTest3(double speed, double degree,
                                      double leftInches, double rightInches,
                                      double timeoutS, PPE_HardwareNarwhalChassis.DriveMode mode, LinearOpMode curLinearOpMode) {
        super.encoderDrive(speed, degree, leftInches, rightInches, timeoutS, mode, curLinearOpMode);
        // Declares variables that are used for this method

        if (degree < 0) {
            leftInches = degree*((WHEEL_BASE * Math.PI)/360);
            rightInches = -((degree*((WHEEL_BASE * Math.PI)/360)));
        }

        else if (degree > 0) {
            rightInches = degree*((WHEEL_BASE * Math.PI)/360);
            leftInches = -((degree*((WHEEL_BASE * Math.PI)/360)));
        }

        else if (degree == 0) {
            leftInches = leftInches;
            rightInches = rightInches;
        }

        int NewLeftTarget;
        int NewRightTarget;
        int RightPosition;
        int LeftPosition;
        double LeftPower;
        double RightPower;

        // Resets encoders to 0
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Checks to make sure that encoders are reset.
        while(leftDrive.getCurrentPosition() > 1 && rightDrive.getCurrentPosition()> 1){
            sleep(25);
        }


        if (curLinearOpMode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            // Calculates the needed encoder ticks by multiplying a pre-determined amount of CountsPerInches,
            // and the method input gets the actual distance travel in inches
            NewLeftTarget = leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            NewRightTarget = rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            // Gets the current position of the encoders at the beginning of the EncoderDrive method
            RightPosition = rightDrive.getCurrentPosition();
            LeftPosition = leftDrive.getCurrentPosition();
            // Gives the encoders the target.
            leftDrive.setTargetPosition(NewLeftTarget);
            rightDrive.setTargetPosition(NewRightTarget);

            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            while(leftDrive.getCurrentPosition() > 1){
                sleep(15);
            }



            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            runtime.reset();
            // This gets where the motor encoders will be at full position when it will be at full speed.
            double LeftEncoderPositionAtFullSpeed = ((AccelerationInches * (COUNTS_PER_INCH)) + LeftPosition);
            double RightEncoderPositionAtFullSpeed = ((AccelerationInches * (COUNTS_PER_INCH)) + RightPosition);
            boolean Running = true;


            // This gets the absolute value of the encoder positions at full speed - the current speed, and while it's greater than 0, it will continues increasing the speed.
            // This allows the robot to accelerate over a set number of inches, which reduces wheel slippage and increases overall reliability
            while (leftDrive.isBusy() && rightDrive.isBusy() && Running && curLinearOpMode.opModeIsActive()) {
                // While encoders are not at position
                if (((Math.abs(speed)) - (Math.abs(leftDrive.getPower()))) > .05){
                    // This allows the robot to accelerate over a set distance, rather than going full speed.  This reduces wheel slippage and increases reliability.
                    LeftPower = (Range.clip(Math.abs((leftDrive.getCurrentPosition()) / (LeftEncoderPositionAtFullSpeed)), .15, speed));
                    RightPower =(Range.clip(Math.abs((rightDrive.getCurrentPosition()) / (RightEncoderPositionAtFullSpeed)), .15, speed));

                    leftDrive.setPower(LeftPower);
                    rightDrive.setPower(RightPower);

                    //telemetry.addData("In Accel loop CM", + Distance.getDistance(DistanceUnit.CM));
                    telemetry.addData("Accelerating", RightEncoderPositionAtFullSpeed);
                    telemetry.addData("Path1", "Running to %7d :%7d", NewLeftTarget, NewRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                    //telemetry.addData("Sections Complete:", +SectionsCompleted);
                    telemetry.update();
                } else if (Math.abs(NewLeftTarget) - Math.abs(leftDrive.getCurrentPosition()) < -1) {
                    Running = false;
                } else {
                    // Multiplies the speed desired by the direction, which has a value of either 1, or -1, and allows for backwards driving with the ramp up function
                    leftDrive.setPower(speed);
                    rightDrive.setPower(speed);

                    //telemetry.addData("In Reg loop CM", + Distance.getDistance(DistanceUnit.CM));
                    telemetry.addData("Path1", "Running to %7d :%7d", NewLeftTarget, NewRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                    //telemetry.addData("Sections Complete:", +SectionsCompleted);
                    telemetry.update();
                }

                // Display information for the driver.


            }

            // Stops all motion
            // Set power to 0
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void encoderDriveTest2(double speed, double degree,
                                      double leftInches, double rightInches,
                                      double timeoutS, PPE_HardwareNarwhalChassis.DriveMode mode, LinearOpMode curLinearOpMode) {
        super.encoderDrive(speed, degree, leftInches, rightInches, timeoutS, mode, curLinearOpMode);

        double     COUNTS_PER_MOTOR_REV    = 560 ;    //Set for NevRest 20 drive. For 40's change to 1120. For 60's 1680
        double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is the ratio between the motor axle and the wheel
        double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
        double     COUNTS_PER_INCH         = 22.48;

        if (degree < 0) {
            leftInches = -(degree * ((WHEEL_BASE * Math.PI) / 360));
            rightInches = ((degree * ((WHEEL_BASE * Math.PI) / 360)));
        } else if (degree > 0) {
            leftInches = -((degree * ((WHEEL_BASE * Math.PI) / 360)));
            rightInches = degree * ((WHEEL_BASE * Math.PI) / 360);
        } else if (degree == 0) {
            leftInches = leftInches;
            rightInches = rightInches;
        }

        //initialise some variables for the subroutine
        int newLeftTarget;
        int newRightTarget;
        // Ensure that the opmode is still active
        // Determine new target position, and pass to motor controller we only do this in case the encoders are not totally zero'd
        newLeftTarget = (leftDrive.getCurrentPosition() + (int)(-leftInches * COUNTS_PER_INCH));
        newRightTarget = (rightDrive.getCurrentPosition() + (int)(-rightInches * COUNTS_PER_INCH));
        // reset the timeout time and start motion.
        runtime.reset();
        // keep looping while we are still active, and there is time left, and neither set of motors have reached the target
        while ( (runtime.seconds() < timeoutS) &&
                (Math.abs(leftDrive.getCurrentPosition()) > newLeftTarget  &&
                        Math.abs(rightDrive.getCurrentPosition()) > newRightTarget)) {
            double rem = (Math.abs(leftDrive.getCurrentPosition()) + Math.abs(rightDrive.getCurrentPosition()))/2;
            double Nspeed;
            //To Avoid spinning the wheels, this will "Slowly" ramp the motors up over
            //the amount of time you set for this SubRun

            /*double R = runtime.seconds();
            if (R < rampup) {
                double ramp = R / rampup;
                Nspeed = speed * ramp;
            }
//Keep running until you are about two rotations out
            else if(rem > (1000) )
            {
                Nspeed = speed;
            }
            //start slowing down as you get close to the target
            else if(rem > (200) && (speed*.2) > .1) {
                Nspeed = speed * (rem / 1000);
            }
            //minimum speed
            else {
                Nspeed = speed * .2;

            }
            //Pass the seed values to the motors
            leftDrive.setPower(Nspeed);
            rightDrive.setPower(Nspeed);


        }
        // Stop all motion;
        //Note: This is outside our while statement, this will only activate once the time, or distance has been met
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        // show the driver how close they got to the last target
        telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
        telemetry.addData("Path2",  "Running at %7d :%7d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        telemetry.update();
        //setting resetC as a way to check the current encoder values easily
        double resetC = ((Math.abs(leftDrive.getCurrentPosition()) + Math.abs(rightDrive.getCurrentPosition())));
        //Get the motor encoder resets in motion
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //keep waiting while the reset is running
        while (Math.abs(resetC) > 0){
            resetC =  ((Math.abs(leftDrive.getCurrentPosition()) + Math.abs(rightDrive.getCurrentPosition())));
            //idle();
        }
        // switch the motors back to RUN_USING_ENCODER mode
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//give the encoders a chance to switch modes.
        //waitOneFullHardwareCycle();
        //  sleep(250);   // optional pause after each move
    }
*/
}

