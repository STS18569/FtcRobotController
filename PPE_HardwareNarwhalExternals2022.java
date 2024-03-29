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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
public class PPE_HardwareNarwhalExternals2022
{
    private HardwareMap hwMap           =  null;

    public DcMotor  carousel            = null;
    public DcMotor  armSwivel           = null;
    public DcMotor  arm                 = null;

    public CRServo  flapper             = null;

    private static enum ArmPosition {REST, RAISED, TOP, MIDDLE, BOTTOM}

    //Needs to be changed depending on gear ratio (every 20 is 560 (ex: 20:1 is 560))
    private static final double TICKS_PER_DEGREE = 1680./360.;


    /* local OpMode members. */
    private ElapsedTime period = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public PPE_HardwareNarwhalExternals2022() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Define and Initialize Motors
        carousel = hwMap.get(DcMotor.class, "carousel");
        armSwivel = hwMap.get(DcMotor.class, "arm_swivel");
        arm = hwMap.get(DcMotor.class, "arm");
        carousel.setDirection(DcMotor.Direction.FORWARD);
        armSwivel.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        carousel.setPower(0);
        armSwivel.setPower(0);
        arm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armSwivel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize ALL installed servos.
        flapper = hwMap.get(CRServo.class, "flapper");

    }

    public void angularArmDrive(ArmPosition position, double speed, double timeoutS) {
        int armDegrees = 0;

        switch(position) {
            case REST:
                armDegrees = 0;
                break;
            case RAISED:
                armDegrees = 30;
                break;
            case TOP:
                armDegrees = 200;
                break;
            case MIDDLE:
                armDegrees = 220;
                break;
            case BOTTOM:
                armDegrees = 270;
                break;
            default:
        }

        // Determine new target position, and pass to motor controller
        int newArmTarget = (int) (armDegrees * -TICKS_PER_DEGREE);
        arm.setTargetPosition(newArmTarget);

        // Turn On RUN_TO_POSITION
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        arm.setPower(Math.abs(speed));
/*
        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.

 */
        while ((runtime.seconds() < timeoutS) &&
                (arm.isBusy())) {
        }

        // Stop all motion;
        arm.setPower(0);

        // Turn off RUN_TO_POSITION
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // sleep(250);   // optional pause after each move
    }
 }

