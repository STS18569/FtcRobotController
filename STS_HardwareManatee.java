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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
public class  STS_HardwareManatee
{
    /* Public OpMode members. */
    static public final boolean CHASSIS_ONLY = true;
    static public final boolean MECANUM = false;

    public DcMotor  leftDrive           = null;
    public DcMotor  rightDrive          = null;
    public DcMotor  middleDrive         = null;
    public DcMotor  leftFrontDrive      = null;
    public DcMotor  leftBackDrive       = null;
    public DcMotor  rightFrontDrive     = null;
    public DcMotor  rightBackDrive      = null;
    public DcMotor  intake              = null;
    public DcMotor  shooterWheelOne     = null;
    public DcMotor  shooterWheelTwo     = null;
    //public DcMotor wobbleArm = null;

    public Servo    wobbleArm = null;
    public Servo    wobbleClaw   = null;
    public Servo    hopper   = null;
    public Servo    shooterAngler = null;


    public static final double WOBBLE_ARM_MID_SERVO =  -1.0;
    public static final double WOBBLE_CLAW_MID_SERVO       =  0.5;
    public static final double HOPPER_MID_SERVO       =  0.5;
    public static final double SHOOTER_ANGLER_MID_SERVO       =  0.5;
    // public static final double ARM_UP_POWER    =  0.45 ;
    // public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public STS_HardwareManatee(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        if (MECANUM) {
            leftFrontDrive = hwMap.get(DcMotor.class, "left_front_drive");
            leftBackDrive = hwMap.get(DcMotor.class, "left_back_drive");
            rightFrontDrive = hwMap.get(DcMotor.class, "right_front_drive");
            rightBackDrive = hwMap.get(DcMotor.class, "right_back_drive");
        }

        else if (!MECANUM) {
            leftDrive   = hwMap.get(DcMotor.class, "left_drive");
            rightDrive  = hwMap.get(DcMotor.class, "right_drive");
            middleDrive = hwMap.get(DcMotor.class, "middle_drive");
        }

        if (!CHASSIS_ONLY) {
            intake = hwMap.get(DcMotor.class, "intake");
            shooterWheelOne = hwMap.get(DcMotor.class, "shooter_wheel_one");
            shooterWheelTwo = hwMap.get(DcMotor.class, "shooter_wheel_two ");
            //wobbleArm = hwMap.get(DcMotor.class, "wobble_arm");
        }

        if (MECANUM) {
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        }
        else if (!MECANUM) {
            leftDrive.setDirection(DcMotor.Direction.REVERSE);
            rightDrive.setDirection(DcMotor.Direction.FORWARD);
            middleDrive.setDirection(DcMotor.Direction.FORWARD);
        }

        if (!CHASSIS_ONLY) {
            intake.setDirection(DcMotor.Direction.REVERSE);
            shooterWheelOne.setDirection(DcMotor.Direction.FORWARD);
            shooterWheelTwo.setDirection(DcMotor.Direction.FORWARD);
            //wobbleArm.setDirection(DcMotor.Direction.FORWARD);
        }

        if (MECANUM) {
            // Set all motors to zero power
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
        }
        else if (!MECANUM) {
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            middleDrive.setPower(0);
        }

        if (!CHASSIS_ONLY) {
            intake.setPower(0);
            shooterWheelOne.setPower(0);
            shooterWheelTwo.setPower(0);
            //wobbleArm.setPower(0);
        }
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        if (MECANUM) {
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else if (!MECANUM) {
            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            middleDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (!CHASSIS_ONLY) {
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterWheelOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterWheelTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //wobbleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Define and initialize ALL installed servos.
            wobbleArm = hwMap.get(Servo.class, "wobble_arm");
            wobbleClaw = hwMap.get(Servo.class, "wobble_claw");
            hopper = hwMap.get(Servo.class, "hopper");
        }
/*
        wobbleArm.setPosition(1.0);
        wobbleClaw.setPosition(WOBBLE_CLAW_MID_SERVO);
        shooterAngler.setPosition(SHOOTER_ANGLER_MID_SERVO);
 */
    }
 }

