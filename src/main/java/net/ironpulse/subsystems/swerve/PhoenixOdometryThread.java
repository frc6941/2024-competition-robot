package net.ironpulse.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static net.ironpulse.Constants.SwerveConstants.*;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 */
public class PhoenixOdometryThread extends Thread {
    private final Lock signalLock = new ReentrantLock();
    private BaseStatusSignal[] signals = new BaseStatusSignal[0];
    private final List<Queue<Double>> queues = new ArrayList<>();
    private final List<Queue<Double>> timestampQueues = new ArrayList<>();
    private boolean isCANFD = false;

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
        if (timestampQueues.isEmpty()) return;
        super.start();
    }

    public Queue<Double> registerSignal(ParentDevice device, StatusSignal<Double> signal) {
        var queue = new ArrayDeque<Double>(100);
        signalLock.lock();
        SwerveSubsystem.odometryLock.lock();
        try {
            isCANFD = CANBus.isNetworkFD(device.getNetwork());
            var newSignals = new BaseStatusSignal[signals.length + 1];
            System.arraycopy(signals, 0, newSignals, 0, signals.length);
            newSignals[signals.length] = signal;
            signals = newSignals;
            queues.add(queue);
        } finally {
            signalLock.unlock();
            SwerveSubsystem.odometryLock.unlock();
        }
        return queue;
    }

    public Queue<Double> makeTimestampQueue() {
        var queue = new ArrayDeque<Double>(100);
        SwerveSubsystem.odometryLock.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            SwerveSubsystem.odometryLock.unlock();
        }
        return queue;
    }

    @Override
    public void run() {
        while (true) {
            signalLock.lock();
            try {
                if (isCANFD) {
                    BaseStatusSignal.waitForAll(2.0 /
                            ODOMETRY_FREQUENCY, signals);
                } else {
                    Thread.sleep((long) (1000.0 / ODOMETRY_FREQUENCY));
                    if (signals.length > 0) BaseStatusSignal.refreshAll(signals);
                }
            } catch (InterruptedException exception) {
                exception.printStackTrace();
            } finally {
                signalLock.unlock();
            }

            SwerveSubsystem.odometryLock.lock();

            try {
                double timestamp = Logger.getRealTimestamp() / 1e6;
                double totalLatency = 0.0;
                for (BaseStatusSignal signal : signals) {
                    totalLatency += signal.getTimestamp().getLatency();
                }
                if (signals.length > 0) {
                    timestamp -= totalLatency / signals.length;
                }
                for (int i = 0; i < signals.length; i++) {
                    queues.get(i).offer(signals[i].getValueAsDouble());
                }
                for (Queue<Double> timestampQueue : timestampQueues) {
                    timestampQueue.offer(timestamp);
                }
            } finally {
                SwerveSubsystem.odometryLock.unlock();
            }
        }
    }
}
