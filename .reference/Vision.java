package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.apriltag.*;

import com.acmerobotics.dashboard.config.Config;
import java.util.List;

@Config
public class Vision {

    /** Two-point linear calibration: real = (raw - DISTANCE_OFFSET) * DISTANCE_SCALE */
    public static double DISTANCE_SCALE  = 1.074;
    public static double DISTANCE_OFFSET = 23.3; // inches

    private Limelight3A limelight;
    private boolean active = false;

    // ID тегов для разных альянсов
    public static final int RED_ALLIANCE_TAG = 24;   // AprilTag для красного альянса
    public static final int BLUE_ALLIANCE_TAG = 20;  // AprilTag для синего альянса

    private int targetTagId = RED_ALLIANCE_TAG; // По умолчанию красный

    // Кэширование для оптимизации - предотвращает множественные запросы к Limelight за один loop
    private FiducialResult cachedTargetFiducial = null;
    private long cacheTimestamp = 0;
    private static final long CACHE_VALIDITY_MS = 20; // 50Hz update rate

    // Обнаружение отключения камеры: если данные устарели — камера зависла/отключилась
    private static final long CAMERA_DISCONNECT_TIMEOUT_MS = 500;

    // Vision Persistence Filter для устранения flickering на больших расстояниях
    private static final int FRAMES_TO_CONFIRM = 3;  // Требуется 3 последовательных фрейма
    private int consecutiveDetections = 0;  // Счетчик последовательных обнаружений
    private int consecutiveLosses = 0;      // Счетчик последовательных потерь
    private boolean stableTagVisible = false;  // Стабильное состояние видимости
    private long lastCounterUpdateTimestamp = -1; // Предотвращает двойное обновление за луп

    public void init(HardwareMap hw) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // Request data 100 times per second
        limelight.start();
    }

    public void start() {
        active = true;
    }

    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
        active = false;
    }

    public boolean isActive() {
        return active;
    }

    /**
     * Устанавливает целевой tag для отслеживания
     * @param isRedAlliance true для красного альянса, false для синего
     */
    public void setAlliance(boolean isRedAlliance) {
        targetTagId = isRedAlliance ? RED_ALLIANCE_TAG : BLUE_ALLIANCE_TAG;
        // Инвалидируем кэш при смене альянса
        cachedTargetFiducial = null;
        cacheTimestamp = 0;
        // Сбрасываем persistence filter
        resetPersistenceFilter();
    }

    /**
     * Внутренний helper: получает FiducialResult для текущего alliance tag
     * Использует кэширование для предотвращения множественных запросов за один loop цикл
     */
    private FiducialResult getTargetFiducial() {
        long now = System.currentTimeMillis();

        // Возвращаем кэшированное значение если оно свежее
        if (cachedTargetFiducial != null && (now - cacheTimestamp) < CACHE_VALIDITY_MS) {
            return cachedTargetFiducial;
        }

        // Обновляем кэш
        cachedTargetFiducial = null;
        cacheTimestamp = now;

        if (!active) return null;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        // Если данные устарели >500ms — камера отключилась, но getLatestResult()
        // продолжает отдавать старый кэш. getStaleness() возвращает возраст данных в ms.
        if (result.getStaleness() > CAMERA_DISCONNECT_TIMEOUT_MS) return null;

        List<FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) return null;

        // Фильтруем по targetTagId (24 для RED, 20 для BLUE)
        // ВАЖНО: Явно игнорируем теги другого альянса для предотвращения tracking неправильных целей
        for (FiducialResult f : fiducials) {
            int fiducialId = f.getFiducialId();

            // ТОЛЬКО наш alliance tag
            if (fiducialId == targetTagId) {
                cachedTargetFiducial = f;
                break;
            }

            // Явно игнорируем тег противоположного альянса
            int oppositeAllianceTag = (targetTagId == RED_ALLIANCE_TAG) ? BLUE_ALLIANCE_TAG : RED_ALLIANCE_TAG;
            if (fiducialId == oppositeAllianceTag) {
                // Пропускаем этот тег - он от другого альянса
                continue;
            }
        }

        return cachedTargetFiducial;
    }

    /**
     * Получает целевой AprilTag (для текущего альянса)
     * Возвращает синтетический AprilTagDetection для обратной совместимости
     */
    public AprilTagDetection getTargetTag() {
        FiducialResult fiducial = getTargetFiducial();
        if (fiducial == null) return null;

        return wrapFiducialAsAprilTag(fiducial);
    }

    /**
     * Преобразует FiducialResult в AprilTagDetection для совместимости с legacy code
     */
    private AprilTagDetection wrapFiducialAsAprilTag(FiducialResult fiducial) {
        // Позиция тега относительно робота (Robot → Tag)
        Pose3D pose = fiducial.getTargetPoseRobotSpace();
        if (pose == null) return null;
        Position pos = pose.getPosition();

        // 3D Euclidean distance
        double range = Math.sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);
        double yaw = fiducial.getTargetXDegrees();
        double pitch = fiducial.getTargetYDegrees();

        // Создаем AprilTagPoseFtc с правильным конструктором
        AprilTagPoseFtc ftcPose = new AprilTagPoseFtc(
            pos.x,                  // x
            pos.y,                  // y
            pos.z,                  // z
            yaw,                    // yaw
            0.0,                    // roll (не используется)
            pitch,                  // pitch
            range,                  // range (3D distance)
            yaw,                    // bearing (approximation)
            pitch                   // elevation (approximation)
        );

        // Создаем AprilTagDetection с правильным конструктором
        return new AprilTagDetection(
            fiducial.getFiducialId(),   // id
            0,                          // hamming
            0.0f,                       // decisionMargin
            null,                       // center
            null,                       // corners
            null,                       // metadata
            ftcPose,                    // ftcPose
            null,                       // rawPose
            pose,                       // fieldPosition (Pose3D)
            System.nanoTime()           // frameAcquisitionNanoTime
        );
    }

    /**
     * Проверяет, виден ли целевой tag (с persistence фильтром)
     *
     * Persistence Filter:
     * - Требуется 3 последовательных фрейма для подтверждения обнаружения/потери
     * - Устраняет single-frame flickering на расстояниях 400-500см
     * - ~60ms задержка при 50Hz loop rate (приемлемо для стабильности)
     */
    public boolean hasTargetTag() {
        boolean rawDetection = getTargetFiducial() != null;

        // Обновляем счетчики только один раз за каждый Limelight poll (cacheTimestamp меняется раз в луп).
        // Без этого: 2+ вызовов за луп → счётчик растёт в 2x быстрее → фильтр "3 фрейма" срабатывает за 1-2 фрейма.
        if (cacheTimestamp != lastCounterUpdateTimestamp) {
            lastCounterUpdateTimestamp = cacheTimestamp;
            if (rawDetection) {
                consecutiveDetections++;
                consecutiveLosses = 0;
            } else {
                consecutiveLosses++;
                consecutiveDetections = 0;
            }
        }

        // Переключаемся на "виден" только после 3 последовательных обнаружений
        if (consecutiveDetections >= FRAMES_TO_CONFIRM) {
            stableTagVisible = true;
        }

        // Переключаемся на "не виден" только после 3 последовательных потерь
        if (consecutiveLosses >= FRAMES_TO_CONFIRM) {
            stableTagVisible = false;
        }

        return stableTagVisible;
    }

    /**
     * Сбрасывает persistence filter (для тестирования или смены альянса)
     */
    public void resetPersistenceFilter() {
        consecutiveDetections = 0;
        consecutiveLosses = 0;
        stableTagVisible = false;
        lastCounterUpdateTimestamp = -1;
    }

    /**
     * Получает расстояние до целевого тега в ДЮЙМАХ
     * Использует встроенный 3D Euclidean distance от Limelight 3A
     * Результат в дюймах для совместимости с Pedro Pathing odometry
     *
     * @return Расстояние в дюймах, или -1 если тег не найден
     */
    public double getTargetDistance() {
        FiducialResult target = getTargetFiducial();
        if (target == null) return -1;

        // Try to get 3D pose from Limelight 3A
        Pose3D pose = target.getTargetPoseRobotSpace();
        if (pose == null) {
            // Fallback: use trigonometric calculation with ty angle
            return calculateDistanceFromAngle(target);
        }

        Position pos = pose.getPosition();
        if (pos == null) {
            // Fallback: use trigonometric calculation
            return calculateDistanceFromAngle(target);
        }

        // Horizontal ground distance (ignore y/height component)
        // distance = sqrt(x² + z²)
        double rangeMeters = Math.sqrt(
            pos.x * pos.x +
            pos.z * pos.z
        );

        // Two-point linear calibration: real = (raw - DISTANCE_OFFSET) * DISTANCE_SCALE
        double rawInches = rangeMeters * 39.3701;
        double distanceInches = (rawInches - DISTANCE_OFFSET) * DISTANCE_SCALE;

        return distanceInches;
    }

    /**
     * Fallback метод: расчет расстояния по углу ty (trigonometry)
     * Используется если Pose3D недоступен
     */
    private double calculateDistanceFromAngle(FiducialResult target) {
        double ty = target.getTargetYDegrees();

        // Измеренные высоты (нужно откалибровать!)
        double cameraHeightInches = 12.0;  // Высота камеры от пола
        double tagHeightInches = 16.0;     // Высота AprilTag от пола
        double heightDiffInches = Math.abs(tagHeightInches - cameraHeightInches);

        // Защита от деления на ноль
        if (Math.abs(ty) < 1.0) {
            return 100.0; // Примерное значение если угол очень маленький
        }

        // distance = heightDiff / tan(ty)
        double distanceInches = heightDiffInches / Math.tan(Math.toRadians(ty));

        return Math.abs(distanceInches);
    }

    /**
     * DEBUG: Расширенная информация о Pose3D для диагностики distance calculation
     */
    public String getDebugPose3DInfo() {
        FiducialResult target = getTargetFiducial();
        if (target == null) return "No target";

        StringBuilder debug = new StringBuilder();

        // Проверка RobotSpace Pose
        Pose3D robotSpace = target.getTargetPoseRobotSpace();
        if (robotSpace == null) {
            debug.append("RobotSpace: NULL\n");
        } else {
            Position pos = robotSpace.getPosition();
            if (pos == null) {
                debug.append("RobotSpace: OK, Position: NULL\n");
            } else {
                double distMeters = Math.sqrt(pos.x*pos.x + pos.y*pos.y + pos.z*pos.z);
                double distInches = distMeters * 39.3701;
                debug.append(String.format("RobotSpace: X:%.2fm Y:%.2fm Z:%.2fm\n", pos.x, pos.y, pos.z));
                debug.append(String.format("  Distance: %.2f in (%.1f cm)\n", distInches, distInches * 2.54));
            }
        }

        // Note: Limelight FiducialResult только предоставляет RobotSpace pose
        // FieldSpace coordinates не доступны напрямую через API

        return debug.toString().trim();
    }

    /**
     * DEBUG: Получает все доступные данные из Limelight для диагностики
     */
    public String getDebugLimelightData() {
        if (!active) return "Vision not active";

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return "No valid result";

        StringBuilder debug = new StringBuilder();

        // FiducialResult for target tag (используем только методы Limelight)
        FiducialResult target = getTargetFiducial();
        if (target != null) {
            double tx = target.getTargetXDegrees(); // Horizontal offset
            double ty = target.getTargetYDegrees(); // Vertical offset

            debug.append(String.format("Tag ID=%d | tx=%.2f° | ty=%.2f°",
                target.getFiducialId(), tx, ty));
        } else {
            debug.append("Target tag not found in fiducials list");
        }

        return debug.toString();
    }

    /**
     * Получает yaw ошибку для целевого тега (для turret)
     * Возвращает горизонтальный угол в градусах (tx - horizontal offset)
     * ВАЖНО: Это прямое значение из Limelight без обработки
     */
    public double getTargetYaw() {
        FiducialResult target = getTargetFiducial();
        if (target == null) return Double.NaN;

        // getTargetXDegrees() - это tx (horizontal offset в градусах)
        // Limelight возвращает: отрицательное = слева, положительное = справа
        double tx = target.getTargetXDegrees();
        return tx;
    }

    /**
     * Получает вертикальный угол (ty) до целевого тега
     */
    public double getTargetPitch() {
        FiducialResult target = getTargetFiducial();
        if (target == null) return Double.NaN;
        return target.getTargetYDegrees();
    }

    /**
     * Получает позу тега в robot space (camera frame) для вычисления полевых координат цели.
     * X = вбок (вправо = +), Z = вперёд от камеры (в метрах)
     * Используется в Turret.updateGoalFromVision() для обновления goalPose по наблюдению.
     *
     * @return Pose3D в robot space, или null если тег не виден
     */
    public Pose3D getTagRobotSpacePose() {
        FiducialResult target = getTargetFiducial();
        if (target == null) return null;
        return target.getTargetPoseRobotSpace();
    }

    /**
     * Получает ближайший AprilTag (любого альянса) для testing/fallback
     */
    public AprilTagDetection getBestTag() {
        if (!active) return null;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        List<FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;

        FiducialResult closest = null;
        double closestDist = Double.MAX_VALUE;

        for (FiducialResult f : fiducials) {
            Pose3D pose = f.getRobotPoseTargetSpace();
            Position pos = pose.getPosition();
            double dist = Math.sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);

            if (dist < closestDist) {
                closestDist = dist;
                closest = f;
            }
        }

        return (closest != null) ? wrapFiducialAsAprilTag(closest) : null;
    }

    /**
     * Проверяет наличие любого видимого тега
     */
    public boolean hasTarget() {
        return getBestTag() != null;
    }

    // Геттеры для телеметрии
    public int getTargetTagId() {
        return targetTagId;
    }

    public String getAllianceColor() {
        return (targetTagId == RED_ALLIANCE_TAG) ? "RED" : "BLUE";
    }
}
