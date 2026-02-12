# Code Analysis and System Report

## Code Analysis

The system `atis_sistem.py` was created by integrating logic from the provided reference files:

1.  **Hardware Control (`olcak.py`)**:
    *   The `HardwareController` class manages low-level hardware interactions.
    *   **Stepper Motor**: Implemented using `Jetson.GPIO` on pins 33 (STEP), 31 (DIR), and 37 (ENA), matching `olcak.py`.
    *   **Servos & BLDC**: Implemented using `adafruit_servokit` on I2C address 0x41. Channels 1, 2, 3 are used for ball dropping, and Channel 4 is used for the Brushless Motor (ESC), matching `olcak.py`.
    *   **Relay**: A placeholder pin (12) is assigned for the water spray relay, as the original file did not specify a pin number.

2.  **Camera & AI (`IDA1.py` & `kamera.py`)**:
    *   The `CameraHandler` class handles the ZED 2i camera and Object Detection.
    *   **ZED 2i**: Initialized using `pyzed.sl` with parameters extracted from `kamera.py` (HD720, 30FPS, PERFORMANCE depth mode).
    *   **YOLO (TensorRT)**: Utilizes the `ultralytics` library but strictly enforces the use of the **TensorRT Engine** format (`.engine`) as seen in `IDA1.py`.
        *   It looks for the engine file at `/home/yarkin/roboboatIDA/roboboat/weights/small640.engine`.
        *   If the engine file is found, it loads it and enables **CUDA** and **FP16** (Half Precision) optimization for the Jetson Orin Nano.
        *   A fallback to `yolov11n.pt` is included only for simulation/testing purposes if the engine file is missing.

3.  **PID Control (`denemePro.py`)**:
    *   The `PIDController` class implements the Yaw PID logic extracted from `denemePro.py`.
    *   It calculates the error in degrees based on pixel offset and applies P, I, and D terms to determine the required stepper movement.

## Scenario Analysis

The system operates in a continuous loop with the following states:

1.  **SCANNING (Default State)**:
    *   The Stepper Motor rotates continuously in small increments to scan the area.
    *   The BLDC motor spins continuously to be ready for firing.
    *   The Relay is set to HIGH (armed).

2.  **Target Detection**:
    *   If **Target ID 2** is detected (Priority High): The system switches to `TRACKING` mode. Once aligned (error < tolerance), it triggers `ACTION_ID2`.
    *   If **Target ID 11** is detected (Priority Low): The system switches to `TRACKING` mode. Once aligned, it triggers `ACTION_ID11`.
    *   **Target Locking**: During actions (especially water spraying), the system locks onto the specific target ID to prevent swapping to a higher priority target mid-action.

3.  **Action Scenarios**:
    *   **Scenario A (ID 2 - Ball Firing)**: The system fires one ball by rotating the next available servo (1, 2, or 3) to 90 degrees and back. It then resumes scanning.
    *   **Scenario B (ID 11 - Water Spraying)**: The system maintains aim (using PID) for 10 seconds to spray the target. After the timer expires, it resumes scanning.

## Adjustable Parameters

The following parameters in `atis_sistem.py` can be tuned for field performance:

### PID Control
*   `KP_YAW` (Default: 0.8): Proportional gain. Increase for faster response, decrease if oscillating.
*   `KI_YAW` (Default: 0.005): Integral gain. Fixes steady-state error.
*   `KD_YAW` (Default: 0.02): Derivative gain. Dampens overshoot.
*   `PID_LIMIT` (Default: 20.0): Limits the integral term accumulation.

### Hardware
*   `STEP_DELAY` (Default: 0.0004): Controls the speed of the Stepper Motor. Lower is faster.
*   `STEPS_PER_REV` (Default: 1600): Steps per full revolution (depends on microstepping settings).
*   `RELAY_PIN`: **MUST BE CONFIGURED**. Currently set to 12.

### Operation
*   `AIM_TOLERANCE_PIXELS` (Default: 20): How close to the center the target must be to trigger an action.
*   `WATER_SPRAY_DURATION` (Default: 10.0): Duration in seconds to hold aim for Scenario B.
*   `SCAN_STEP_SIZE` (Default: 5): How many steps to move per loop iteration during scanning.
*   `MODEL_PATH`: Path to the TensorRT `.engine` file.


to turkish :
Görev Analizi: Görev 4 - İkmal Teslimatı (Supply Drop) vs. atis_sistem.pyGörev 4 - İkmal Teslimatı için yaptığınız yeni tanıma dayanarak, mevcut atis_sistem.py kodunda önemli eksiklikler bulunmaktadır. Mekanik kısımlar (top fırlatma, su sıkma, nişan alma) uygulanmış olsa da, tespit mantığı ve operasyonel kurallar farklıdır.Mevcut kodun Görev 4 için neden yetersiz olduğunun ve nelerin değişmesi gerektiğinin dökümü aşağıdadır:1. Hedef Tanımlama UyuşmazlığıMevcut Kod:ID 2'yi (Araba/Top) Tespit Eder $\rightarrow$ Top Fırlatır.ID 11'i (Dur İşareti/Su) Tespit Eder $\rightarrow$ Su Sıkar.Görev 4 Gereksinimi:Sarı Tekne + Siyah Üçgen $\rightarrow$ Su Teslimatı (Püskürtme).Siyah Tekne + Siyah Artı $\rightarrow$ Nesne Teslimatı (Top Fırlatma).Eksiklik: Mevcut YOLO modeli (arabalar/dur işaretleri gibi COCO sınıfları üzerinde eğitilmiş), muhtemelen "Sarı Tekne", "Siyah Üçgen", "Siyah Tekne" veya "Artı İşareti"ni doğru şekilde tespit etmeyecektir.Çözüm: Bu özel şamandıraları/şekilleri tanımlamak için özel olarak eğitilmiş bir YOLO modeline (veya çok spesifik bir renk/şekil algılama algoritmasına) ihtiyacınız var. Mevcut class_id mantığı (2 ve 11) bu görev için yanlıştır.2. Eylem Mantığı FarklılıklarıSu Teslimatı (Sarı Tekne):Mevcut: 10 saniye boyunca su sıkar.Görev 4: Hedefi en az 3 saniye boyunca vurmalıdır.Karar: Mevcut 10 saniyelik zamanlayıcı yeterlidir (minimum 3 saniye gereksinimini karşılar), ancak israfa neden olabilir.Nesne Teslimatı (Siyah Tekne):Mevcut: Algılama sırası başına bir top ateşler.Görev 4: Tekneye bir raket topu teslim eder. İDA (İnsansız Deniz Aracı) önceden 3 top ile yüklenebilir.Karar: Mevcut mantık (servo 1, sonra 2, sonra 3 döndür) uyumludur, ancak tetikleme koşulu ("Siyah Artı"yı görmek) eksiktir.3. Operasyonel BağlamMevcut kod, bir alanı tarayan sabit bir taret varsayar.Görev 4, bir parkurda hareket eden bir İDA (İnsansız Deniz Aracı) gerektirir.Eksiklik: atis_sistem.py tareti kontrol eder, ancak teknenin navigasyon sistemiyle iletişim kurmaz.Tekne bir hedef gördüğünde duruyor mu?Taret teknenin hareketini telafi ediyor mu?Mevcut PID basittir (piksel hatasıyla orantılı). Hareket halindeki bir teknede, ileri beslemeli (feed-forward) kontrole veya teknenin IMU/Hız verileriyle daha sıkı bir entegrasyona ihtiyacınız olabilir.
