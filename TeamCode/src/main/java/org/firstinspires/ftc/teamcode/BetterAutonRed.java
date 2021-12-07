package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Better Auton RED", group="Iterative Opmode")
public class BetterAutonRed extends OpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    public CalibrateGyro cbgyro;
    private BNO055IMU imu;
    private DcMotor spinning;

    private int stage = 0;
    private ElapsedTime runtime = new ElapsedTime();


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
        spinning = hardwareMap.get(DcMotor.class, "spinning");

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

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        switch (stage) {
            case 0: // reset and initialization
                resetEncoders();
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                stage++;
                break;

            case 1: // if the motor is given a target number of tics, it goes that amount of tics
                strafeLeft(127);
                runToPosition();
                setMotorPower(0.2);

                stage++;
                runtime.reset();
                break;

            case 2:
                if (Math.abs(leftFront.getCurrentPosition() - leftFront.getTargetPosition()) <= 20) {
                    resetEncoders();
                    runtime.reset();
                    moveReverse(585);
                    stage++;
                }
                break;

            case 3:
                if (runtime.milliseconds() > 500) {
                    runToPosition();
                }

                if (Math.abs(leftFront.getCurrentPosition() - leftFront.getTargetPosition()) <= 20) {
                    resetEncoders();
                    strafeLeft(700);

                    stage++;
                }
                break;

            case 4: //run duck spinner
                if (runtime.milliseconds()<3500) {
                    spinning.setPower(-0.7);
                }
                else {
                    spinning.setPower(0);
                    stage++;
                }
                break;


            case 5:
                if (runtime.milliseconds() > 500) {
                    runToPosition();
                }

                if (Math.abs(leftFront.getCurrentPosition() - leftFront.getTargetPosition()) <= 20) {
                    resetEncoders();
                    moveReverse(200);
                    stage++;
                }
                break;

            case 6:
                if (runtime.milliseconds() > 500) {
                    runToPosition();
                }

                if (Math.abs(leftFront.getCurrentPosition() - leftFront.getTargetPosition()) <= 20) {
                    resetEncoders();
                    stage++;
                }
                break;

        }

        telemetry.addData("STAGE ", stage);
        telemetry.update();
    }

    @Override
    public void stop() {
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
    }


    int distanceToTicks(double x) {
        return (int) ((x / 301.59) * 537.7);
    }

    public void setMotorPower(double power) {
        rightFront.setPower(power);
        leftFront.setPower(power);
        rightBack.setPower(power);
        leftBack.setPower(power);
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

    public void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        runtime.reset();
    }

    public void runToPosition() {
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
