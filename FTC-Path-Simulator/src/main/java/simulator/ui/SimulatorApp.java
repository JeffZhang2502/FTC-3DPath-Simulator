package simulator.ui;

import javafx.animation.AnimationTimer;
import javafx.geometry.Insets;
import javafx.scene.*;
import javafx.scene.control.*;
import javafx.scene.layout.*;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.*;
import javafx.scene.text.Font;
import javafx.scene.text.FontWeight;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import javafx.stage.FileChooser;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import simulator.algorithm.PathFinder;
import simulator.engine.AutoSimulationEngine;
import simulator.io.JavaCodeParser;
import simulator.model.*;
import simulator.physics.BallProjectileEngine;

/**
 * 3D JavaFX UI for the FTC Auto Simulator.
 *
 * <p>Replaces the old 2D Canvas with a PerspectiveCamera-driven
 * SubScene featuring 3D boxes, spheres, and real-time ball trajectory.</p>
 */
public class SimulatorApp {

    // ---- coordinate mapping ----
    //  World X → JavaFX X
    //  World Y → JavaFX Z
    //  World height → JavaFX -Y

    // ---- model ----
    private final FieldMap field = new FieldMap();
    private RobotProfile profile = new RobotProfile();
    private Robot robot;
    private AutoSimulationEngine engine;

    // ---- 3D scene ----
    private static final double FIELD_HALF = FieldMap.HALF_FIELD;   // 72.0

    private SubScene subScene;
    private PerspectiveCamera camera;
    private Group worldRoot;

    // camera orbit
    private double camAzim = 45, camElev = 40, camDist = 140;
    private double lastMX, lastMY;
    private Group camPivot, camTilt;

    // materials
    private final PhongMaterial matFloor   = mat(Color.rgb(50, 50, 65));
    private final PhongMaterial matTileA   = mat(Color.rgb(70, 70, 85));
    private final PhongMaterial matTileB   = mat(Color.rgb(60, 60, 75));
    private final PhongMaterial matRobot   = mat(Color.rgb(70, 200, 70));
    private final PhongMaterial matDir     = mat(Color.rgb(255, 255, 0));
    private final PhongMaterial matObelisk = mat(Color.rgb(60, 60, 70));
    private final PhongMaterial matDivider = mat(Color.rgb(100, 100, 110));
    private final PhongMaterial matWall    = mat(Color.rgb(160, 160, 170));
    private final PhongMaterial matGoal    = mat(Color.rgb(40, 40, 55));
    private final PhongMaterial matZoneR   = mat(Color.rgb(255, 80, 80, 0.35));
    private final PhongMaterial matZoneB   = mat(Color.rgb(80, 80, 255, 0.35));
    private final PhongMaterial matClassif = mat(Color.rgb(200, 140, 60, 0.7));
    private final PhongMaterial matTrail   = mat(Color.rgb(255, 255, 100, 0.6));
    private final PhongMaterial matPath    = mat(Color.rgb(50, 255, 80, 0.7));
    private final PhongMaterial matBall    = mat(Color.rgb(255, 215, 0));
    private final PhongMaterial matBallTr  = mat(Color.rgb(255, 200, 50, 0.7));

    // scene-graph references
    private Group robotGroup;
    private Box robotBody;
    private Sphere robotDirDot;
    private Group trailGroup = new Group();
    private Group pathGroup   = new Group();
    private Group ballGroup   = new Group();
    private Group ballTrailGroup = new Group();

    // ---- UI components ----
    private final BorderPane root = new BorderPane();
    private final Label statusLabel = new Label("[READY] Configure robot and press Start.");

    private final TextField widthField  = new TextField("18.0");
    private final TextField lengthField = new TextField("18.0");
    private final CheckBox visionCheck  = new CheckBox();
    private final ComboBox<String> intakeCombo  = new ComboBox<>();
    private final ComboBox<String> scoreCombo   = new ComboBox<>();
    private final ComboBox<String> chassisMotorCombo = new ComboBox<>();
    private final ComboBox<String> launcherMotorCombo = new ComboBox<>();
    private final TextArea programArea = new TextArea();
    private final Button startBtn = new Button("▶ Start 30s Auto Simulation");
    private final Button resetBtn = new Button("↺ Reset");
    private final Button uploadBtn = new Button("📁 Upload .java OpMode");

    // ---- simulation ----
    private AnimationTimer animTimer;
    private long lastFrameNanos;
    private boolean running;

    // path rendering cache
    private List<Pose2d> lastPath = List.of();

    // ---- public ----

    public SimulatorApp() {
        this.robot = new Robot(profile, new Pose2d(-48, 48, 0));
        this.engine = new AutoSimulationEngine(robot, field);
        buildUI();
        build3DScene();
        updateCamera();
    }

    public BorderPane getRoot() { return root; }

    // ======================== 2D UI ========================

    private void buildUI() {
        Label title = new Label("FTC Auto Simulator — DECODE 2025-2026   [3D]");
        title.setFont(Font.font("SansSerif", FontWeight.BOLD, 18));
        title.setPadding(new Insets(10, 16, 10, 16));
        root.setTop(title);
        root.setLeft(buildLeftPanel());

        // 3D sub-scene in center.
        StackPane centerPane = new StackPane();
        centerPane.setStyle("-fx-background-color: #111;");
        root.setCenter(centerPane);

        // Status bar.
        statusLabel.setFont(Font.font("Monospaced", 13));
        statusLabel.setPadding(new Insets(6, 16, 6, 16));
        statusLabel.setStyle("-fx-background-color: #eee; -fx-text-fill: #333;");
        root.setBottom(statusLabel);
    }

    private VBox buildLeftPanel() {
        VBox panel = new VBox(8);
        panel.setPadding(new Insets(12));
        panel.setPrefWidth(290);
        panel.setStyle("-fx-background-color: #f5f5f5;");

        // Dimensions
        Label dimTitle = boldLabel("Robot Dimensions (inches)");
        GridPane dimGrid = grid(8, 4);
        dimGrid.add(new Label("Width:"), 0, 0);   dimGrid.add(widthField, 1, 0);
        dimGrid.add(new Label("Length:"), 0, 1);  dimGrid.add(lengthField, 1, 1);

        // Motors
        Label motTitle = boldLabel("Motor Hardware");
        chassisMotorCombo.getItems().addAll(
            "GOBILDA_5203_19_2", "GOBILDA_5203_30_1", "REV_HD_HEX_20_1");
        chassisMotorCombo.setValue("GOBILDA_5203_19_2");
        launcherMotorCombo.getItems().addAll(
            "GOBILDA_5203_19_2", "GOBILDA_5203_30_1", "REV_HD_HEX_20_1");
        launcherMotorCombo.setValue("GOBILDA_5203_19_2");

        GridPane motGrid = grid(8, 4);
        motGrid.add(new Label("Chassis:"), 0, 0);   motGrid.add(chassisMotorCombo, 1, 0);
        motGrid.add(new Label("Launcher:"), 0, 1);  motGrid.add(launcherMotorCombo, 1, 1);

        // Features
        Label featTitle = boldLabel("Hardware Features");
        visionCheck.setText("Vision Sensor (Camera)");
        intakeCombo.getItems().addAll("NONE", "SERVO_CLAW", "INTAKE_ROLLER");
        intakeCombo.setValue("NONE");
        scoreCombo.getItems().addAll("NONE", "OBELISK_ONLY", "CLASSIFIER_ONLY");
        scoreCombo.setValue("NONE");

        GridPane featGrid = grid(8, 4);
        featGrid.add(new Label("Intake:"), 0, 0);  featGrid.add(intakeCombo, 1, 0);
        featGrid.add(new Label("Scoring:"), 0, 1); featGrid.add(scoreCombo, 1, 1);

        // Program
        Label progTitle = boldLabel("Auto Program");
        programArea.setPrefRowCount(8);
        programArea.setFont(Font.font("Monospaced", 11));
        programArea.setText("MOVE_TO(24, 24)\nINTAKE\nMOVE_TO(-48, -12)\nLAUNCH\n");

        // Buttons
        startBtn.setPrefWidth(260);
        startBtn.setStyle("-fx-background-color: #2e7d32; -fx-text-fill: white; "
                        + "-fx-font-weight: bold; -fx-font-size: 13px;");
        startBtn.setOnAction(e -> startSimulation());

        resetBtn.setPrefWidth(260);
        resetBtn.setOnAction(e -> resetSimulation());

        uploadBtn.setPrefWidth(260);
        uploadBtn.setOnAction(e -> uploadOpMode());

        panel.getChildren().addAll(
            dimTitle, dimGrid, new Separator(),
            motTitle, motGrid, new Separator(),
            featTitle, visionCheck, featGrid, new Separator(),
            progTitle, programArea, new Separator(),
            startBtn, resetBtn, uploadBtn);
        return panel;
    }

    private void uploadOpMode() {
        FileChooser fc = new FileChooser();
        fc.setTitle("Open FTC OpMode .java file");
        fc.getExtensionFilters().add(
            new FileChooser.ExtensionFilter("Java Files", "*.java"));
        File f = fc.showOpenDialog(root.getScene().getWindow());
        if (f == null) return;

        try {
            List<String> cmds = new JavaCodeParser().parseFile(f.toPath());
            if (cmds.isEmpty()) {
                statusLabel.setText("[WARN] No FTC commands detected in file.");
                return;
            }
            StringBuilder sb = new StringBuilder();
            for (String c : cmds) sb.append(c).append('\n');
            programArea.setText(sb.toString().trim());
            statusLabel.setText("[OK] Loaded " + cmds.size() + " commands from "
                                + f.getName());
        } catch (IOException ex) {
            statusLabel.setText("[ERROR] Failed to read file: " + ex.getMessage());
        }
    }

    // ======================== 3D SCENE ========================

    private void build3DScene() {
        worldRoot = new Group();

        // --- lighting ---
        AmbientLight ambient = new AmbientLight(Color.rgb(80, 80, 90));
        PointLight point = new PointLight(Color.rgb(220, 220, 230));
        point.setTranslateX(0); point.setTranslateY(-100); point.setTranslateZ(0);

        // --- floor ---
        Box floor = box(144, 0.1, 144, matFloor);
        floor.setTranslateY(0.05);
        worldRoot.getChildren().add(floor);

        // --- 6×6 tiles ---
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                Box tile = box(24, 0.02, 24, (i + j) % 2 == 0 ? matTileA : matTileB);
                tile.setTranslateX(-FIELD_HALF + 12 + i * 24);
                tile.setTranslateZ(-FIELD_HALF + 12 + j * 24);
                tile.setTranslateY(0.11);
                worldRoot.getChildren().add(tile);
            }
        }

        // --- field elements (height from official CAD / manual) ---
        for (FieldMap.FieldElement el : field.getElements()) {
            double cx = (el.minX + el.maxX) / 2.0;
            double cz = (el.minY + el.maxY) / 2.0;
            double w  = el.maxX - el.minX;
            double d  = el.maxY - el.minY;
            double h  = el.height > 0.05 ? el.height : 0.15;

            PhongMaterial m = switch (el.type) {
                case OBELISK         -> matObelisk;
                case CLASSIFIER_BOX, CLASSIFIER_RAMP -> matClassif;
                case PERIMETER_WALL  -> matWall;
                case GOAL_ASSEMBLY   -> matGoal;
                case OBSTACLE        -> matDivider;
                case RED_START       -> matZoneR;
                case BLUE_START      -> matZoneB;
                default -> matZoneR;
            };

            // Obelisk → triangular prism mesh, not a box.
            if (el.type == FieldMap.ZoneType.OBELISK) {
                MeshView tri = createTriangularPrism(
                    (float) FieldMap.OBELISK_FACE_WIDTH,
                    (float) FieldMap.OBELISK_HEIGHT,
                    m);
                tri.setTranslateX(FieldMap.OBELISK_CX);
                tri.setTranslateZ(FieldMap.OBELISK_CY);
                tri.setTranslateY(h / 2.0f);   // centre at half-height above ground
                worldRoot.getChildren().add(tri);
                // Semi-transparent collision-approximation cylinder.
                Cylinder obCol = new Cylinder(FieldMap.OBELISK_CIRCRADIUS, h);
                obCol.setMaterial(mat(Color.rgb(60, 60, 70, 0.3)));
                obCol.setTranslateX(FieldMap.OBELISK_CX);
                obCol.setTranslateZ(FieldMap.OBELISK_CY);
                obCol.setTranslateY(h / 2.0);
                worldRoot.getChildren().add(obCol);
                continue;
            }

            Box b = box(w, h, d, m);
            b.setTranslateX(cx);
            b.setTranslateZ(cz);
            b.setTranslateY(h / 2.0);   // centre half-height above ground
            worldRoot.getChildren().add(b);
        }

        // --- robot group ---
        robotGroup = new Group();
        double rw = profile.width, rl = profile.length, rh = profile.height;
        robotBody = box(rw, rh, rl, matRobot);
        robotBody.setTranslateY(rh / 2.0);
        robotDirDot = new Sphere(1.5);
        robotDirDot.setMaterial(matDir);
        robotDirDot.setTranslateX(rl / 2.0);
        robotDirDot.setTranslateY(rh - 1.0);
        robotGroup.getChildren().addAll(robotBody, robotDirDot);
        updateRobotTransform();
        worldRoot.getChildren().add(robotGroup);

        // --- trail + path + ball groups ---
        worldRoot.getChildren().addAll(trailGroup, pathGroup, ballGroup, ballTrailGroup);

        // --- root group ---
        Group sceneRoot = new Group(worldRoot, ambient, point);

        // --- camera with orbit pivot groups ---
        camera = new PerspectiveCamera(true);
        camera.setFieldOfView(45);
        camera.setNearClip(1);
        camera.setFarClip(800);
        camera.setTranslateZ(-camDist);
        camTilt = new Group(camera);
        camPivot = new Group(camTilt);
        sceneRoot.getChildren().add(camPivot);

        // --- sub-scene ---
        subScene = new SubScene(sceneRoot, 700, 700, true, SceneAntialiasing.BALANCED);
        subScene.setCamera(camera);

        // size binding
        StackPane center = (StackPane) root.getCenter();
        subScene.widthProperty().bind(center.widthProperty());
        subScene.heightProperty().bind(center.heightProperty());
        center.getChildren().add(subScene);

        // mouse controls
        subScene.setOnMousePressed(e -> { lastMX = e.getSceneX(); lastMY = e.getSceneY(); });
        subScene.setOnMouseDragged(e -> {
            double dx = e.getSceneX() - lastMX;
            double dy = e.getSceneY() - lastMY;
            lastMX = e.getSceneX(); lastMY = e.getSceneY();
            camAzim -= dx * 0.3;
            camElev += dy * 0.3;
            camElev = Math.max(5, Math.min(85, camElev));
            updateCamera();
        });
        subScene.setOnScroll(e -> {
            camDist -= e.getDeltaY() * 0.2;
            camDist = Math.max(60, Math.min(300, camDist));
            updateCamera();
        });
    }

    private void updateCamera() {
        camera.setTranslateZ(-camDist);
        camTilt.getTransforms().setAll(new Rotate(-camElev, Rotate.X_AXIS));
        camPivot.getTransforms().setAll(new Rotate(-camAzim, Rotate.Y_AXIS));
    }

    // ======================== PER-FRAME UPDATE ========================

    private void update3D() {
        updateRobotTransform();
        updateTrail();
        updatePath();
        updateBall();
    }

    private void updateRobotTransform() {
        double rx = robot.currentPose.x;
        double rz = robot.currentPose.y;
        robotGroup.getTransforms().clear();
        robotGroup.getTransforms().add(new Translate(rx, 0, rz));
        robotGroup.getTransforms().add(new Rotate(
            -Math.toDegrees(robot.currentPose.heading), 0, 1, 0));
    }

    private void updateTrail() {
        trailGroup.getChildren().clear();
        List<Pose2d> trail = engine.getTrail();
        if (trail.size() < 2) return;
        for (int i = 0; i < trail.size(); i += 2) {   // skip every other for perf
            Pose2d p = trail.get(i);
            Sphere s = new Sphere(0.8);
            s.setMaterial(matTrail);
            s.setTranslateX(p.x);
            s.setTranslateZ(p.y);
            s.setTranslateY(0.5);
            trailGroup.getChildren().add(s);
        }
    }

    private void updatePath() {
        List<Pose2d> path = engine.getCurrentPath();
        if (path.equals(lastPath)) return;
        lastPath = new ArrayList<>(path);

        pathGroup.getChildren().clear();
        if (path.size() < 2) return;
        for (int i = 0; i < path.size(); i++) {
            Pose2d p = path.get(i);
            Box b = box(1.5, 0.2, 1.5, matPath);
            b.setTranslateX(p.x);
            b.setTranslateZ(p.y);
            b.setTranslateY(0.6);
            pathGroup.getChildren().add(b);
        }
    }

    private void updateBall() {
        ballGroup.getChildren().clear();
        ballTrailGroup.getChildren().clear();

        BallProjectileEngine be = engine.getBallEngine();
        if (!be.isActive()) return;

        // ball sphere
        Sphere ball = new Sphere(1.5);
        ball.setMaterial(matBall);
        ball.setTranslateX(be.getX());
        ball.setTranslateZ(be.getY());
        ball.setTranslateY(be.getZ());    // world height → +Y (up)
        ballGroup.getChildren().add(ball);

        // trajectory trail
        List<double[]> traj = be.getTrajectory();
        for (int i = 0; i < traj.size(); i += 3) {
            double[] pt = traj.get(i);
            Sphere s = new Sphere(0.6);
            s.setMaterial(matBallTr);
            s.setTranslateX(pt[0]);
            s.setTranslateZ(pt[1]);
            s.setTranslateY(pt[2]);   // world height → +Y (up)
            ballTrailGroup.getChildren().add(s);
        }
    }

    // ======================== SIMULATION CONTROL ========================

    private void startSimulation() {
        if (running) return;
        try {
            double w = Double.parseDouble(widthField.getText().trim());
            double l = Double.parseDouble(lengthField.getText().trim());
            if (w <= 0 || l <= 0 || w > 30 || l > 30) {
                statusLabel.setText("[ERROR] Dimensions must be 1–30 inches.");
                return;
            }
            profile = new RobotProfile(
                w, l, 8.0,
                MotorType.valueOf(chassisMotorCombo.getValue()),
                2.0,
                MotorType.valueOf(launcherMotorCombo.getValue()),
                45.0, 1.5,
                visionCheck.isSelected(),
                RobotProfile.IntakeType.valueOf(intakeCombo.getValue()),
                RobotProfile.ScoringCapability.valueOf(scoreCombo.getValue()));
        } catch (Exception ex) {
            statusLabel.setText("[ERROR] Invalid config: " + ex.getMessage());
            return;
        }

        robot = new Robot(profile, new Pose2d(-48, 48, 0));
        engine = new AutoSimulationEngine(robot, field);
        engine.loadProgram(programArea.getText());
        engine.reset();
        engine.loadProgram(programArea.getText());

        // Refresh robot body size.
        robotGroup.getChildren().clear();
        double rw = profile.width, rl = profile.length, rh = profile.height;
        robotBody = box(rw, rh, rl, matRobot);
        robotBody.setTranslateY(rh / 2.0);
        robotDirDot = new Sphere(1.5);
        robotDirDot.setMaterial(matDir);
        robotDirDot.setTranslateX(rl / 2.0);
        robotDirDot.setTranslateY(rh - 1.0);
        robotGroup.getChildren().addAll(robotBody, robotDirDot);
        trailGroup.getChildren().clear();
        pathGroup.getChildren().clear();
        ballGroup.getChildren().clear();
        ballTrailGroup.getChildren().clear();
        lastPath = List.of();

        running = true;
        lastFrameNanos = 0;
        startBtn.setDisable(true);
        statusLabel.setStyle("-fx-background-color: #e8f5e9; -fx-text-fill: #2e7d32;");

        animTimer = new AnimationTimer() {
            @Override
            public void handle(long nowNanos) {
                if (lastFrameNanos == 0) { lastFrameNanos = nowNanos; return; }
                double dt = (nowNanos - lastFrameNanos) / 1_000_000_000.0;
                lastFrameNanos = nowNanos;

                engine.tick(dt);
                statusLabel.setText(engine.getStatusText());
                update3D();

                if (engine.isFinished()) stopAnimation();
            }
        };
        animTimer.start();
    }

    private void stopAnimation() {
        if (animTimer != null) animTimer.stop();
        running = false;
        startBtn.setDisable(false);
        statusLabel.setStyle("-fx-background-color: #fff3e0; -fx-text-fill: #e65100;");
        List<String> recent = engine.getRecentLog(5);
        StringBuilder sb = new StringBuilder(engine.getStatusText());
        for (String s : recent) sb.append("  |  ").append(s);
        statusLabel.setText(sb.toString());
    }

    private void resetSimulation() {
        if (running) stopAnimation();
        robot = new Robot(profile, new Pose2d(-48, 48, 0));
        engine = new AutoSimulationEngine(robot, field);
        engine.loadProgram(programArea.getText());
        trailGroup.getChildren().clear();
        pathGroup.getChildren().clear();
        ballGroup.getChildren().clear();
        ballTrailGroup.getChildren().clear();
        lastPath = List.of();
        updateRobotTransform();
        statusLabel.setText("[READY] Simulation reset. Press Start to run.");
        statusLabel.setStyle("-fx-background-color: #eee; -fx-text-fill: #333;");
    }

    // ======================== HELPERS ========================

    private static PhongMaterial mat(Color c) {
        PhongMaterial m = new PhongMaterial();
        m.setDiffuseColor(c);
        m.setSpecularColor(c.brighter());
        return m;
    }

    private static Box box(double w, double h, double d, PhongMaterial m) {
        Box b = new Box(w, h, d);
        b.setMaterial(m);
        return b;
    }

    /**
     * Builds an equilateral triangular prism MeshView (for the obelisk §9.6).
     *
     * @param s face width in inches (11")
     * @param h prism height (23")
     * @param m PhongMaterial
     * @return MeshView centred at local origin, base on Y=0 plane
     */
    private static MeshView createTriangularPrism(float s, float h, PhongMaterial m) {
        float R = (float) (s / Math.sqrt(3.0));       // circumradius ≈ 6.35"
        float halfH = h / 2f;

        // 6 vertices: bottom triangle (Y=+halfH) + top triangle (Y=−halfH)
        // JavaFX convention: positive Y = up in our scene.
        float[] pts = {
            // bottom triangle (Y = +halfH)
             R,          halfH,    0f,           // v0
            -R / 2f,     halfH,    R * 0.8660254f,  // v1 (R·√3/2 ≈ 5.5")
            -R / 2f,     halfH,   -R * 0.8660254f,  // v2
            // top triangle (Y = −halfH)
             R,         -halfH,    0f,           // v3
            -R / 2f,    -halfH,    R * 0.8660254f,  // v4
            -R / 2f,    -halfH,   -R * 0.8660254f   // v5
        };

        float[] tex = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        int[] faces = {
            // bottom (looking from below): ccw = 0,2,1
            0,0, 2,0, 1,0,
            // top (looking from above): ccw = 3,4,5
            3,0, 4,0, 5,0,
            // side 0-1
            0,0, 1,0, 4,0,
            0,0, 4,0, 3,0,
            // side 1-2
            1,0, 2,0, 5,0,
            1,0, 5,0, 4,0,
            // side 2-0
            2,0, 0,0, 3,0,
            2,0, 3,0, 5,0
        };

        TriangleMesh mesh = new TriangleMesh();
        mesh.getPoints().setAll(pts);
        mesh.getTexCoords().setAll(tex);
        mesh.getFaces().setAll(faces);

        MeshView mv = new MeshView(mesh);
        mv.setMaterial(m);
        mv.setCullFace(CullFace.NONE);
        return mv;
    }

    private static Label boldLabel(String t) {
        Label l = new Label(t);
        l.setFont(Font.font("SansSerif", FontWeight.BOLD, 13));
        return l;
    }

    private static GridPane grid(int hgap, int vgap) {
        GridPane g = new GridPane();
        g.setHgap(hgap);
        g.setVgap(vgap);
        return g;
    }
}
