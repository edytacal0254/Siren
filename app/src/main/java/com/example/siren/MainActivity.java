package com.example.siren;

import androidx.appcompat.app.AppCompatActivity;

import org.apache.commons.math3.filter.DefaultMeasurementModel;
import org.apache.commons.math3.filter.DefaultProcessModel;
import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;


import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.media.AudioFormat;
import android.media.AudioManager;
import android.media.AudioTrack;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.widget.CompoundButton;
import android.widget.Switch;


public class MainActivity extends AppCompatActivity implements SensorEventListener {
    // SENSOR_DELAY_GAME - (20,000 microsecond delay)
    // delays are not exactly the same, when specified delay rate, sampling is usually faster
    private final int SAMPLING_PERIOD_US = 20000;
    private final int US_TO_S = 1000000;

    volatile double velocity = 0.0d;
    volatile boolean running = false;

    private SensorManager sensorManager;
    private Sensor sensor;

    private HandlerThread sensorHandlerThread;
    private Handler sensorHandler;

    private HandlerThread audioHandlerThread;
    private Handler audioHandler;

    private AudioTrack audioTrack;

    Switch aSwitch;

    // delta t
    private final double dt = ((double) SAMPLING_PERIOD_US) / ((double) US_TO_S);

    // audio params
    private final int sampleRate = 8000;
    private final int numSamples = (int) (dt * sampleRate);
    private final double[] sample = new double[numSamples];
    private final double freqOfTone = 100; // hz
    private final byte[] generatedSnd = new byte[2 * numSamples];

    // velocity measurement noise meter/sec
    private final double measurement_noise = 0.25d;
    private final double mnp2 = Math.pow(measurement_noise, 2);

    // A - state transition matrix
    RealMatrix A;

    // B - control input matrix
    RealMatrix B;

    // H -measurement matrix
    RealMatrix H;

    RealVector x;

    // Q - process noise covariance matrix
    RealMatrix Q;

    // R - measurement noise covariance matrix
    RealMatrix ones_3x3 = new Array2DRowRealMatrix(new double[][]  {{1, 1, 1},
            {1, 1, 1},
            {1, 1, 1}});
    RealMatrix R_matrix;

    // P - error covariance matrix
    RealMatrix P0;

    ProcessModel pm;
    MeasurementModel mm;
    KalmanFilter kalmanFilter;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        aSwitch = findViewById(R.id.switch1);

        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        sensor = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);

        audioTrack = new AudioTrack(AudioManager.STREAM_MUSIC,
                8000, AudioFormat.CHANNEL_OUT_MONO,
                AudioFormat.ENCODING_PCM_16BIT, numSamples,
                AudioTrack.MODE_STREAM);

        Runnable r = new Runnable() {
            @Override
            public void run() {
                playSound();
                genTone(velocity);
                if(running) {
                    audioHandler.post(this);
                }
            }
        };

        aSwitch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            synchronized public void onCheckedChanged(CompoundButton compoundButton, boolean b) {
                if(b) {
                    init_kalman_filter();
                    velocity = 0.0d;
                    running = true;

                    sensorHandlerThread = new HandlerThread("Sensor thread", Thread.MAX_PRIORITY);
                    sensorHandlerThread.start();
                    sensorHandler = new Handler(sensorHandlerThread.getLooper());
                    sensorManager.registerListener(MainActivity.this, sensor, SAMPLING_PERIOD_US, sensorHandler);

                    audioHandlerThread = new HandlerThread("tune handler thread");
                    audioHandlerThread.start();
                    audioHandler = new Handler(audioHandlerThread.getLooper());
                    audioHandler.post(r);
                }else{
                    running = false;

                    sensorManager.unregisterListener(MainActivity.this, sensor);

                    sensorHandlerThread.quitSafely();
                    audioHandlerThread.quitSafely();
                }
            }
        });

    }

    @Override
    synchronized public void onSensorChanged(SensorEvent sensorEvent) {
        double a_x = sensorEvent.values[0];
        double a_y = sensorEvent.values[1];
        double a_z = sensorEvent.values[2];

        RealVector z = new ArrayRealVector(new double[] { a_x, a_y, a_z});

        kalmanFilter.predict();
        kalmanFilter.correct(z);

        double v_x = kalmanFilter.getStateEstimation()[0];
        double v_y = kalmanFilter.getStateEstimation()[1];
        double v_z = kalmanFilter.getStateEstimation()[2];

        velocity = Math.sqrt(Math.pow(v_x, 2) + Math.pow(v_y, 2) + Math.pow(v_z, 2));
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {
    }

    private void init_kalman_filter(){
        // A - state transition matrix
        A = new Array2DRowRealMatrix(new double[][] {
                {1, 0, 0},
                {0, 1, 0},
                {0, 0, 1}});

        // B - control input matrix
        B = new Array2DRowRealMatrix(new double[][] {
                {dt, 0 , 0},
                {0, dt, 0},
                {0, 0, dt}});

        // H - measurement matrix
        H = new Array2DRowRealMatrix(new double[][] {
                {1, 0, 0},
                {0, 1, 0},
                {0, 0, 1}});

        x = new ArrayRealVector(new double[] {0, 0, 0});

        // Q - process noise covariance matrix
        Q = new Array2DRowRealMatrix(new double[][] {
                {Math.pow(dt,2), 0, 0},
                {0, Math.pow(dt,2), 0},
                {0, 0, Math.pow(dt,2)}});

        // R - measurement noise covariance matrix
        R_matrix = ones_3x3.copy().scalarMultiply(mnp2);

        // P - error covariance matrix
        P0 = ones_3x3.copy();

        pm = new DefaultProcessModel(A, B, Q, x, P0);
        mm = new DefaultMeasurementModel(H, R_matrix);
        kalmanFilter = new KalmanFilter(pm, mm);
    }

    void genTone(double velocity){
        // fill out the array
        for (int i = 0; i < numSamples; ++i) {
            sample[i] = Math.sin(2 * Math.PI * i / (sampleRate/(freqOfTone*velocity)));
        }

        // convert to 16 bit pcm sound array
        // assumes the sample buffer is normalised.
        int idx = 0;
        for (final double dVal : sample) {
            // scale to maximum amplitude
            final short amp = (short) Math.min((short) velocity * 546,(short)32767);
            final short val = (short) ((dVal * amp));
            // in 16 bit wav PCM, first byte is the low order byte
            generatedSnd[idx++] = (byte) (val & 0x00ff);
            generatedSnd[idx++] = (byte) ((val & 0xff00) >>> 8);
        }
    }

    void playSound(){
        audioTrack.write(generatedSnd, 0, generatedSnd.length);
        audioTrack.play();
    }

}