package application;

import java.util.concurrent.*;
import javax.inject.Inject;
import com.kuka.task.ITaskLogger;

public class OCPUA_cycle {

    @Inject
    private ITaskLogger logger;

    private ScheduledFuture<?> opcuaBackgroundCycle;
    private ScheduledExecutorService scheduler;

    @Inject    
    private OPCUA_Client_Manager opcua;

    private boolean initialized = false;

    public void initialize() {
        if (initialized) {
            logger.warn("OCPUA_cycle already initialized.");
            return;
        }
        initialized = true;

        scheduler = Executors.newSingleThreadScheduledExecutor();

        try {
            opcua.setUp();
        } catch (Exception e) {
            logger.error("Error during OPCUA setup", e);
        }
    }

    public void startCycle() {
        if (scheduler == null || scheduler.isShutdown()) {
            logger.error("Scheduler not available. Did you call initialize()?");
            return;
        }

        Runnable backgroundTask = new Runnable() {
            @Override
            public void run() {
                if (!opcua.isDisconnectRequested) {
                    try {
                        opcua.serverUpdate();
                    } catch (Exception e) {
                        logger.error("Error in OPCUA background cycle", e);
                    }
                } else {
                    logger.info("OPCUA disconnected, shutting down cycle.");
                    stop();
                }
            }
        };

        opcuaBackgroundCycle = scheduler.scheduleAtFixedRate(
            backgroundTask, 
            0, 
            50, 
            TimeUnit.MILLISECONDS
        );
    }

    public void stop() {
        try {
            if (opcuaBackgroundCycle != null) {
            	opcuaBackgroundCycle.cancel(true);
            }
            if (scheduler != null) {
                scheduler.shutdownNow();
                try {
                    if (!scheduler.awaitTermination(5, TimeUnit.SECONDS)) {
                        logger.warn("Scheduler did not terminate in time.");
                    }
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        } catch (Exception e) {
            logger.warn("Error while shutting down scheduler: " + e.getMessage());
        }

        // Always attempt to disconnect client
        try {
            if (opcua != null) {
            	opcua.clientDisconnect();
            }
        } catch (Exception e) {
            logger.warn("Error while disconnecting OPC UA client: " + e.getMessage());
        }
    }

}
