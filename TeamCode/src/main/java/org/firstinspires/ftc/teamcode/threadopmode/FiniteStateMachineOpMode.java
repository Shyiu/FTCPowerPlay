package org.firstinspires.ftc.teamcode.threadopmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.List;

/**
 * A type of {@link OpMode} that contains threads to be ran in parallel periodically.
 * Register threads with {@link ThreadOpMode#registerThread(TaskThread)}
 */
public abstract class ThreadOpMode extends OpMode {
    private List<TaskThread> threads = new ArrayList<>();

    /**
     * Registers a new {@link TaskThread} to be ran periodically.
     * Registered threads will automatically be started during {@link OpMode#start()} and stopped during {@link OpMode#stop()}.
     *
     * @param taskThread A {@link TaskThread} object to be ran periodically.
     */
    public final void registerThread(TaskThread taskThread) {
        threads.add(taskThread);
    }

    /**
     * Contains code to be ran before the OpMode is started. Similar to {@link OpMode#init()}.
     */
    public abstract void mainInit();
    /**
     * Contains code to be ran periodically in the MAIN thread. Similar to {@link OpMode#loop()}.
     */
    public abstract void mainLoop();

    /**
     * Should not be called by subclass.
     */
    @Override
    public final void init() {
        mainInit();
    }

    /**
     * Should not be called by subclass.
     */
    @Override
    public final void start() {
        for(TaskThread taskThread : threads) {
            taskThread.start();
        }
    }

    /**
     * Should not be called by subclass.
     */
    @Override
    public final void loop() {
        mainLoop();
    }

    /**
     * Should not be called by subclass.
     */
    @Override
    public final void stop() {
        for(TaskThread taskThread : threads) {
            taskThread.stop();
        }
    }
}