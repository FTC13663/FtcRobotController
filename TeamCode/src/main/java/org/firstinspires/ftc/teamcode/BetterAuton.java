package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Parking", group="Iterative Opmode")
public class BetterAuton extends OpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    public CalibrateGyro cbgyro;
    private BNO055IMU imu;

    private int stage = 0;
    private int step = 10;
    private ElapsedTime runtime = new ElapsedTime();
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
    public void init() {

    }

    @Override
    public void loop() {

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
