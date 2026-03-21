// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.Logger;

/**
 * Provides an interface for asynchronously reading high-frequency measurements
 * to a set of queues.
 *
 * <p>
 * This version is intended for Phoenix 6 devices on both the RIO and CANivore
 * buses. It refreshes all registered signals at a fixed rate, estimates a
 * shared sample timestamp from the FPGA clock minus average CAN latency, and
 * publishes the sampled values into small queues consumed by the subsystem IO
 * layers.
 */
public class PhoenixOdometryThread extends Thread {
    private static final int SAMPLE_QUEUE_CAPACITY = 20;

    private final Lock signalsLock = new ReentrantLock(); // Prevents conflicts when registering signals
    private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
    private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
    private final List<Queue<Double>> timestampQueues = new ArrayList<>();
    private long droppedPhoenixSamples = 0L;
    private long droppedTimestampSamples = 0L;

    private static PhoenixOdometryThread instance = null;

    public static PhoenixOdometryThread getInstance() {
        if (instance == null) {
            instance = new PhoenixOdometryThread();
        }
        return instance;
    }

    private PhoenixOdometryThread() {
        setName("PhoenixOdometryThread");
        setDaemon(true);
    }

    @Override
    public void start() {
        if (timestampQueues.size() > 0) {
            super.start();
        }
    }

    /** Registers a Phoenix signal to be read from the thread. */
    public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(SAMPLE_QUEUE_CAPACITY);
        signalsLock.lock();
        Drive.odometryLock.lock();
        try {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
            System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
            newSignals[phoenixSignals.length] = signal;
            phoenixSignals = newSignals;
            phoenixQueues.add(queue);
        } finally {
            signalsLock.unlock();
            Drive.odometryLock.unlock();
        }
        return queue;
    }

    /** Returns a new queue that returns timestamp values for each sample. */
    public Queue<Double> makeTimestampQueue() {
        Queue<Double> queue = new ArrayBlockingQueue<>(SAMPLE_QUEUE_CAPACITY);
        Drive.odometryLock.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            Drive.odometryLock.unlock();
        }
        return queue;
    }

    @Override
    public void run() {
        while (true) {
            // Sleep outside the lock so signal registration is never blocked by the wait.
            try {
                Thread.sleep((long) (1000.0 / Drive.ODOMETRY_FREQUENCY));
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            // Refresh signals under signalsLock to prevent concurrent modification,
            // but hold the lock only for the refresh itself.
            signalsLock.lock();
            try {
                if (phoenixSignals.length > 0) {
                    BaseStatusSignal.refreshAll(phoenixSignals);
                }
            } finally {
                signalsLock.unlock();
            }

            // Save new data to queues
            Drive.odometryLock.lock();
            try {
                // Sample timestamp is current FPGA time minus average CAN latency.
                // Default timestamps from Phoenix are NOT compatible with
                // FPGA timestamps; this solution is imperfect but close.
                double timestamp = RobotController.getFPGATime() / 1e6;
                double totalLatency = 0.0;
                for (BaseStatusSignal signal : phoenixSignals) {
                    totalLatency += signal.getTimestamp().getLatency();
                }
                if (phoenixSignals.length > 0) {
                    timestamp -= totalLatency / phoenixSignals.length;
                }

                // Add new samples to queues
                for (int i = 0; i < phoenixSignals.length; i++) {
                    if (!offerLatestSample(phoenixQueues.get(i), phoenixSignals[i].getValueAsDouble())) {
                        droppedPhoenixSamples++;
                    }
                }
                for (int i = 0; i < timestampQueues.size(); i++) {
                    if (!offerLatestSample(timestampQueues.get(i), timestamp)) {
                        droppedTimestampSamples++;
                    }
                }
                Logger.recordOutput("Drive/OdometryThread/DroppedPhoenixSamples", droppedPhoenixSamples);
                Logger.recordOutput("Drive/OdometryThread/DroppedTimestampSamples", droppedTimestampSamples);
            } finally {
                Drive.odometryLock.unlock();
            }
        }
    }

    private static boolean offerLatestSample(Queue<Double> queue, double value) {
        if (queue.offer(value)) {
            return true;
        }
        queue.poll();
        return queue.offer(value);
    }
}
