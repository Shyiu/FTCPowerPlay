package org.firstinspires.ftc.teamcode;

public class TaskThread {

    Actions actions;
    int delay;
    TaskRunnable taskRunnable;

    /** Creates an object that contains code to be ran periodically on a separate thread
     * Can be registered to a {@link ThreadOpMode} with {@link ThreadOpMode#registerThread(TaskThread)}
     *
     * @param actions An {@link Actions} interface that contains code to run periodically
     *                The default delay between runs is 0 milliseconds
     */
    public TaskThread(Actions actions) {
        this(0, actions);
    }

    /** Creates an object that contains code to be ran periodically on a separate thread.
     * Can be registered to a {@link ThreadOpMode} with {@link ThreadOpMode#registerThread(TaskThread)}.
     *
     * @param delay The delay in milliseconds between each periodic run of the code.
     * @param actions An {@link Actions} interface that contains code to run periodically.
     */
    public TaskThread(int delay, Actions actions) {
        this.actions = actions;
        this.delay = delay;
        this.taskRunnable = new TaskRunnable();
    }

    /**
     * An interface to be passed to a {@link TaskThread} constructor.
     */
    public interface Actions {
        /**
         * Robot code to be ran periodically on its own thread.
         */
        public void loop();
    }

    void start() {
        taskRunnable.start();
    }

    void stop() {
        taskRunnable.stop();
    }

    class TaskRunnable implements Runnable {
        private Thread t;

        TaskRunnable() {

        }

        public void run() {
            try {
                while(!t.isInterrupted()) {
                    actions.loop();
                    Thread.sleep(delay);
                }
            } catch (InterruptedException e) {

            }
        }

        public void start() {
            if (t == null) {
                t = new Thread(this);
                t.start();
            }
        }

        public void stop() {
            t.interrupt();
        }
    }
}