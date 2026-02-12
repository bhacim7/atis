import time
import board
import busio
import Jetson.GPIO as GPIO
from adafruit_servokit import ServoKit

# --- 1. GPIO AYARLARI (Step Motor İçin) ---
GPIO.cleanup()
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

STEP_PIN = 33
DIR_PIN = 31
ENA_PIN = 37
#su tetik mekanizması için yeni bir pin de gelecek sadece gpio high veya low yapılacak ama biz hep high kullanacağız

# Pin Kurulumu
GPIO.setup([STEP_PIN, DIR_PIN, ENA_PIN], GPIO.OUT, initial=GPIO.LOW)
# ENA LOW = Motor Aktif (TMC2208 kuralı)
GPIO.output(ENA_PIN, GPIO.LOW)

# --- 2. I2C AYARLARI (Bus 0 - Pin 27 & 28) ---
try:
    # Jetson Orin Nano Bus 0 tanımlaması
    try:
        i2c_bus0 = busio.I2C(board.SCL_1, board.SDA_1)
    except:
        import adafruit_blinka.microcontroller.tegra.t194.pin as pin

        i2c_bus0 = busio.I2C(pin.I2C0_SCL, pin.I2C0_SDA)

    # Adres: 0x41 (Senin lehimlediğin adres)
    kit = ServoKit(channels=16, i2c=i2c_bus0, address=0x41)
    print("PCA9685 (0x41) Bağlandı.")

except Exception as e:
    print(f"Bağlantı Hatası: {e}")
    print("Kabloların Pin 27 (SDA) ve Pin 28 (SCL) olduğuna emin misin?")
    exit()

# --- HASSAS AYARLAR ---
kit.servo[1].set_pulse_width_range(1000, 2000)  # Servo1
kit.servo[2].set_pulse_width_range(1000, 2000)  # Servo2
kit.servo[3].set_pulse_width_range(1000, 2000)  # servo3
kit.servo[4].set_pulse_width_range(1000, 2000)  # esc


# --- FONKSİYONLAR ---

def pwm_to_angle(pwm_degeri):
    """PWM (us) değerini Açıya çevirir."""
    if pwm_degeri < 1000: pwm_degeri = 1000
    if pwm_degeri > 2000: pwm_degeri = 2000
    return (pwm_degeri - 1000) * 180 / 1000


def mermi_sur(tur_sayisi=1):
    """Step motoru belirtilen tur kadar döndürür"""
    adim_sayisi = 1600 * tur_sayisi  # 1600 adım = 1 Tur (1/8 mikrostep varsayımıyla)

    print(f" -> Mermi Sürülüyor ({tur_sayisi} Tur)...")
    GPIO.output(DIR_PIN, 1)  # Yön

    for _ in range(adim_sayisi):
        GPIO.output(STEP_PIN, 1)
        time.sleep(0.0004)  # Hız (Daha düşük sayı = Daha hızlı)
        GPIO.output(STEP_PIN, 0)
        time.sleep(0.0004)


# --- 3. ARMING (MOTORU HAZIRLAMA) ---
print("ESC Arming yapılıyor (Güvenlik için 0 Gaz)...")
kit.servo[4].angle = 0
kit.servo[1].angle = 0
kit.servo[2].angle = 0
kit.servo[3].angle = 0

time.sleep(2)  # Bip-Bip sesi bekle
print("SİSTEM HAZIR!")

try:
    print("--- TAM OTOMATİK MOD BAŞLIYOR ---")

    # A) FIRÇASIZ MOTORU BAŞLAT (SABİT HIZ)
    hedef_pwm = 1850
    esc_acisi = pwm_to_angle(hedef_pwm)
    kit.servo[2].angle = esc_acisi
    print(f"Fırçasız Motor Çalıştı: {hedef_pwm}us")

    # B) DÖNGÜ: NİŞAN AL -> ATEŞ ET
    while True:
        # 1. Pozisyon: 0 Derece
        print("\n--- Hedef 1: 0 Derece ---")
        kit.servo[1].angle = 0
        time.sleep(0.5)  # Servonun varmasını bekle
        mermi_sur(1)  # 1 Tur mermi sür

        # 2. Pozisyon: 90 Derece
        print("\n--- Hedef 2: 90 Derece ---")
        kit.servo[1].angle = 90
        time.sleep(0.5)  # Servonun varmasını bekle
        mermi_sur(1)  # 1 Tur mermi sür

except KeyboardInterrupt:
    print("\nDurduruluyor...")

finally:
    # Çıkarken her şeyi kapat
    kit.servo[2].angle = 0  # Motoru Durdur
    kit.servo[1].angle = 0  # Servoyu sıfırla
    GPIO.output(ENA_PIN, GPIO.HIGH)  # Step motor enerjisini kes
    GPIO.cleanup()
    print("Sistem Güvenli Şekilde Kapatıldı.")