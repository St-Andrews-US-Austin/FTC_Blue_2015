/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Linear Tele Op Mode
 * <p>
 * Enables control of the robot via the gamepad.
 * NOTE: This op mode will not work with the NXT Motor Controllers. Use an Nxt op mode instead.
 */
public class RunRobot extends OpMode {

    DcMotor motorTopR;
    DcMotor motorTopL;
    DcMotor motorBotR;
    DcMotor motorBotL;
    Servo servo;
    Boolean slowMode = false;

    Double servoPos = 0.01;

    @Override
    public void init()  {
        motorBotR = hardwareMap.dcMotor.get("motorBotR");
        motorBotL = hardwareMap.dcMotor.get("motorBotL");
        servo = hardwareMap.servo.get("armTurn");
        servo.setPosition(0.0);
    }
    @Override
    public void start() {

    }
    @Override
    public void loop() {
        if (Math.abs(gamepad1.left_stick_y) > .05 || Math.abs(gamepad1.right_stick_y) > .05) {
            motorBotL.setPower(gamepad1.left_stick_y);
            motorBotR.setPower(-gamepad1.right_stick_y);

        } else {
            motorBotL.setPower(0);
            motorBotR.setPower(0);

        }

        if((Math.abs(gamepad2.left_stick_y) > .05)) {

            servo.setPosition(((gamepad2.left_stick_y) + 1) / 2);
            if(slowMode == true){
                if(gamepad2.left_stick_y > 0){
                    servo.setPosition(0.5);
                }else{
                    servo.setPosition(0.6);
                }
            }
        }else{
            servo.setPosition(0.55);
        }

        System.out.println(servo.getPosition());
        //     }else{
        // servo.setPosition(((gamepad2.left_stick_y) + 1) / 2 + 0.17);

        // }

        if(gamepad2.left_stick_button == true){
            if(slowMode == true){
                slowMode = false;
            }else{
                slowMode = true;
            }
        }
    }
    @Override
    public void stop() {

    }
}
