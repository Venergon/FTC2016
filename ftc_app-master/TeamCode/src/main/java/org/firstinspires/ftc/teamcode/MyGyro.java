package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;

/**
 * Created by Segway on 21-Nov-16.
 */
public class MyGyro extends Activity implements SensorEventListener {
    private SensorManager sensorManager;
    private float angle;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        angle = 0;
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
            getGyro(event);
        }
    }

    private void getGyro(SensorEvent event) {
        float[] values = event.values;
        angle = values[2];
    }

    public float getAngle() {
        return angle;
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }
}
