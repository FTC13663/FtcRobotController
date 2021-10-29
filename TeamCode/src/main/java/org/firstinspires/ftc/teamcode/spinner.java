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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

    @TeleOp(name="Driving_Spinning", group="Iterative Opmode")
// @Disabled
    public class spinner extends OpMode
    {
        // Declare OpMode members.
        private DcMotor leftFront = null;
        private DcMotor leftBack = null;
        private DcMotor rightFront = null;
        private DcMotor rightBack = null;
        private DcMotor spinning;
        //private BNO055IMU imu;

        /*/
         * Code to run ONCE when the driver hits INIT\\\\\\
         */
        @Override
        public void init() {
            telemetry.addData("Status", "Initialized");

            // set up the motors
            leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
            leftBack = hardwareMap.get(DcMotor.class, "leftBack");
            rightFront  = hardwareMap.get(DcMotor.class, "rightFront");
            rightBack = hardwareMap.get(DcMotor.class, "rightBack");
            spinning = hardwareMap.get(DcMotor.class, "spinning");
            //imu = hardwareMap.get(BNO055IMU.class, "imu");

            //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            //parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            //parameters.mode = BNO055IMU.SensorMode.GYRONLY;

            //this.imu.initialize(parameters);


            // we have 4 motors to rotate each mecanum wheel
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            leftBack.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            rightBack.setDirection(DcMotor.Direction.FORWARD);
            spinning.setDirection(DcMotor.Direction.REVERSE);

            // Tell the driver that initialization is complete.
            telemetry.addData("Status", "Initialized");
        }

        @Override
        public void init_loop() {
            // gets data for robot orientation
            //telemetry.addData("imu heading: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        }

        @Override
        public void start() { }

        @Override
        public void loop() {
            // defining the power
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            // this denominator scales the values outside of range [1,-1]
            leftFront.setPower ((y + x + rx) / denominator);
            leftBack.setPower ((y - x + rx) / denominator);
            rightFront.setPower ((y - x - rx) / denominator);
            rightBack.setPower ((y + x - rx) / denominator);

            if (gamepad1.a) {
                spinning.setPower(.7);
            } else if (gamepad1.b) {
                spinning.setPower(-0.7);
            } else {
                spinning.setPower(0);
            }
        }

        /*
         * Code to run ONCE after the driver hits STOP
         */
        @Override
        public void stop() {
            rightFront.setPower(0);
            leftFront.setPower(0);
            rightBack.setPower(0);
            leftBack.setPower(0);
        }


    }

