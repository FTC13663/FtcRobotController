package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class CalibrateGyro {

    private static final String filename = "BNO055IMUCalibration.json";

    public static void initializeGyro(HardwareMap hwMap, String configName, boolean readFromFile, BNO055IMU gyro) {
        gyro = hwMap.get(BNO055IMU.class, configName);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        // If read calibration data from file, set param to filename
        if (readFromFile) {
          parameters.calibrationDataFile = filename;
        }
        // Otherwise log calibration and do not read data from file
        else {
          parameters.calibrationDataFile = null;
        }
        // Initialize gyro with either data from file or new data
        gyro.initialize(parameters);

        // If we aren't reading from a file, write new data to file
        if (!readFromFile) {
          writeCalibrationDataToFile(gyro);
        }
    }

    private static void writeCalibrationDataToFile(BNO055IMU gyro) {
        BNO055IMU.CalibrationData calibrationData = gyro.readCalibrationData();
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, calibrationData.serialize());
    }

    public static List<String> getCalibrationInfo(BNO055IMU gyro) {
        List<String> telemetryData = new ArrayList<>();
        telemetryData.add("Sensor: IMU Gyro --------");
        telemetryData.add("Status: " + gyro.getSystemStatus().toShortString());
        telemetryData.add("Calib Status: " + gyro.getCalibrationStatus().toString());
        telemetryData.add("Gyro Calib? " + gyro.isGyroCalibrated());
        telemetryData.add("heading: " + gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle);

        return telemetryData;
    }
}