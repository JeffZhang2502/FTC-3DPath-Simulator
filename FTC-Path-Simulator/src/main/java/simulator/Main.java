package simulator;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.Scene;
import javafx.stage.Stage;
import simulator.ui.SimulatorApp;

/**
 * FTC Auto Simulator entry point.
 * Launches the JavaFX application window with control panel and field canvas.
 */
public class Main extends Application {

    public static void main(String[] args) {
        launch(args);
    }

    @Override
    public void start(Stage stage) {
        SimulatorApp app = new SimulatorApp();

        Scene scene = new Scene(app.getRoot(), 1020, 740);
        stage.setScene(scene);
        stage.setTitle("FTC Auto Simulator — DECODE 2025-2026");
        stage.setMinWidth(900);
        stage.setMinHeight(600);
        stage.setOnCloseRequest(e -> {
            System.out.println("FTC Simulator shutdown.");
            Platform.exit();
        });
        stage.show();
    }
}
