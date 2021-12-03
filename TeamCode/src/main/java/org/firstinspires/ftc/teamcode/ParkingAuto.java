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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Parking", group="Iterative Opmode")
// @Disabled
public class ParkingAuto extends OpMode {
    // Declare OpMode members.
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    public CalibrateGyro cbgyro;
    private BNO055IMU imu;

    private int stage = 0;
    private int step = 10;
    private ElapsedTime runtime = new ElapsedTime();

    /*/
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // we have 4 motors to rotate each mecanum wheel
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // resets the the encoders to 0 when init is pressed


        // Set targets for drivetrain -- FIRST MOVEMENT
        // motors remain in the same place at 0 power


        cbgyro = new CalibrateGyro(false);
        imu = cbgyro.initGyro(hardwareMap);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        cbgyro.initLoopGyro(imu, telemetry);
        telemetry.addData("BL Enc: ", this.leftBack.getCurrentPosition());
        telemetry.addData("FL Enc: ", this.leftFront.getCurrentPosition());
        telemetry.addData("BR Enc: ", this.rightBack.getCurrentPosition());
        telemetry.addData("FR Enc: ", this.rightFront.getCurrentPosition());
        telemetry.addData("BL Enc: ", this.leftBack.getTargetPosition());
        telemetry.addData("FL Enc: ", this.leftFront.getTargetPosition());
        telemetry.addData("BR Enc: ", this.rightBack.getTargetPosition());
        telemetry.addData("FR Enc: ", this.rightFront.getTargetPosition());
        telemetry.update();

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        switch (stage) {
            case 0:
                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                stage = step;
                break;

            case 1:

                // if the motor is given a target number of tics, it goes that amount of tics
                leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                stage = 2;
                runtime.reset();
                break;

            case 2:
                // forward
                leftFront.setPower(0.5);
                rightFront.setPower(0.5);
                leftBack.setPower(0.5);
                rightBack.setPower(0.5);

                if (Math.abs(leftFront.getCurrentPosition() - leftFront.getTargetPosition()) <= 100) {
                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                    stage = step + 1;
                }
                if (runtime.milliseconds() > 4000)
                {
                    stage = step + 1;
                }
                break;

            case 10:
                moveForward(500);
                stage = 0;
                break;

            case 11:
                moveReverse(500);
                stage = 0;
                break;


            /*case 2:
                leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                runtime.reset();
                stage=3;
                break;*/

            /*case 3:
                leftFront.setPower(0.5);
                rightFront.setPower(-0.5);
                leftBack.setPower(-0.5);
                rightBack.setPower(0.5);
                if (runtime.milliseconds() > 1000) {
                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                    stage=4;
                }
                break;*/
        }

        telemetry.addData("stage: ", stage);
        telemetry.update();
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

    /*
    Distance you want to travel in mm converted to motor ticks
     */
    int distanceToTicks(double x) {
        return (int) ((x / 301.59) * 537.7);
    }

    public void moveForward(double distance) {
        int ticks = distanceToTicks(distance);
        leftFront.setTargetPosition(ticks);
        rightFront.setTargetPosition(ticks);
        leftBack.setTargetPosition(ticks);
        rightBack.setTargetPosition(ticks);
    }

    public void moveReverse(double distance) {
        moveForward(distance * -1);
    }

    public void strafeRight(double distance) {
        int ticks = distanceToTicks(distance);
        leftFront.setTargetPosition(ticks);
        leftBack.setTargetPosition(ticks * -1);
        rightFront.setTargetPosition(ticks * -1);
        rightBack.setTargetPosition(ticks);
    }

    public void strafeLeft(double distance) {
        strafeRight(distance * -1);
    }

}
