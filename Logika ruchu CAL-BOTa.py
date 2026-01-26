import cv2
import time
import random
import board
import busio
import mediapipe as mp
from adafruit_pca9685 import PCA9685
import pigpio


# obiekt I2C do komunikacji z PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

pi = pigpio.pi()
NECK_PIN = 13
pi.set_PWM_frequency(NECK_PIN, 1500)


def angle_pwm(angle):
    pos_us = 500 + (angle / 180) * 2000  # zamiana kąta w µs: 0°=500µs, 180°=2500µs
    pos_pca = int((pos_us / 20000) * 65535)  # 20ms = 50Hz, przeliczamy na 0-65535
    return pos_pca


# przypisanie nazw serw do konkretnych kanałów PCA9685
servo_channels = {
    "LeftRight": 0,
    "UpDown": 1,
    "BotLeft": 3,  # im mniej tym wyzej
    "TopLeft": 2,  # im mniej tym wyzej
    "BotRight": 5,  # im wiecej tym wyzej
    "TopRight": 4,  # im wiecej tym wyzej
    "Neck": 12,
}
# ograniczenia kątów dla każdego serwa (min, max)
tracking_limits = {
    "LeftRight": (60, 120),  # lewo/prawo
    "UpDown": (30, 100),  # góra/dół
    "BotLeft": (90, 150),  # powieka dolna lewa
    "TopLeft": (90, 10),  # powieka górna lewa (odwrotna)
    "BotRight": (90, 30),  # powieka dolna prawa (odwrotna)
    "TopRight": (90, 170),  # powieka górna prawa

}
# limity do mrugania
blink_limits = {
    "TopLeft": (90, 30),
    "BotLeft": (90, 130),
    "TopRight": (90, 140),
    "BotRight": (90, 50)
}
# aktualne pozycje serw
servo_targets = {
    "LeftRight": 90,
    "UpDown": 65,
    "BotLeft": 120,
    "TopLeft": 40,
    "BotRight": 60,
    "TopRight": 140,

}


# funkcja ustawiająca serwa
def set_servo(name, angle):
    min_a, max_a = tracking_limits[name]
    if min_a < max_a:
        angle = max(min_a, min(max_a, angle))  # clamp
    else:
        angle = min(min_a, max(max_a, angle))
    servo_targets[name] = angle  #
    pca.channels[servo_channels[name]].duty_cycle = angle_pwm(angle)


# ustawianie mrugania
def set_servo_blink(name, angle):
    min_a, max_a = blink_limits[name]
    if min_a < max_a:
        angle = max(min_a, min(max_a, angle))
    else:
        angle = min(min_a, max(max_a, angle))
    set_servo(name, angle)
    pca.channels[servo_channels[name]].duty_cycle = angle_pwm(angle)


# wyowałaniue mrugania
def blink():
    Kp= 0.1
    set_servo_blink("TopRight", 90)
    set_servo_blink("BotRight", 90)
    set_servo_blink("TopLeft", 90)
    set_servo_blink("BotLeft", 90)
    time.sleep(0.12)


#  ustawiania limitów od powiek
def lid_limit(lid_value, name):
    min_a, max_a = tracking_limits[name]
    if min_a < max_a:
        return max(min_a, min(max_a, lid_value))
    else:
        return min(min_a, max(max_a, lid_value))


# kalibracja
def calibrate():
    set_servo("LeftRight", eye_x)
    set_servo("UpDown", eye_y)
    set_servo("TopLeft", lid_1)
    set_servo("BotRight", lid_4)
    set_servo("TopRight", lid_2)
    set_servo("BotLeft", lid_3)


# otwieranie oczu
def eyes_open():
    Kp=0.1
    set_servo("TopRight", 150)
    set_servo("BotRight", 120)
    set_servo("TopLeft", 30)
    set_servo("BotLeft", 60)
    time.sleep(1)


# animacja na start - nie uzywana
def lookout():
    eyes_open()
    blink()
    eyes_open()
    blink()
    eyes_open()
    set_servo("LeftRight", 60)
    time.sleep(1)
    blink()
    eyes_open()
    set_servo("LeftRight", 120)
    time.sleep(1)
    blink()
    eyes_open()
    set_servo("LeftRight", 90)

# ustawienia mediapipe
mp_face = mp.solutions.face_detection
face_detection = mp_face.FaceDetection(
    model_selection=0,
    min_detection_confidence=0.6
)

cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

# ustawianie oczu
eye_x = 90  # początkowy kąt w osi X (lewo-prawo)
eye_y = 65  # początkowy kąt w osi Y (góra-dół)
lid_1 = 30  # TopLeft
lid_2 = 150  # TopRight
lid_3 = 120  # BotLeft
lid_4 = 60  # BotRight
DEADZONE_Y = 12  # piksele
DEADZONE_X = 10
OFFSET_X = -50
OFFSET_Y = 20
last_blink = time.time()
next_blink = random.uniform(4, 8)

# parametry regulatora PD
Kp = 0.040
Kd = 0.025
prev_error_x = 0
prev_error_y = 0
smoothing = 1.0
# parametry obrotu głowy
pi.set_servo_pulsewidth(NECK_PIN, 1500)
NECK_STOP = 1500
NECK_LEFT = 1550  # skręt w lewo (im bliżej 1500 tym wolniej)
NECK_RIGHT = 1450  # skręt w prawo
EYE_LEFT_THRESHOLD = 70
EYE_RIGHT_THRESHOLD = 110
current_neck_value = NECK_STOP
STEP_SIZE_UP = 5  # o ile zmieniamy wartość w każdym kroku
STEP_SIZE_DOWN = 10

calibrate()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = face_detection.process(frame_rgb)
    h, w, _ = frame.shape
    cx, cy = w // 2 + OFFSET_X, h // 2 + OFFSET_Y

    if results.detections:
        detection = results.detections[0]
        bbox = detection.location_data.relative_bounding_box
        face_x = int((bbox.xmin + bbox.width / 2) * w)
        face_y = int((bbox.ymin + bbox.height / 2) * h)

        # obliczenie błędu
        error_x = cx - face_x
        error_y = cy - face_y
        diff_x = error_x - prev_error_x
        diff_y = error_y - prev_error_y

        # logika ruchu w osi X (Lewo-Prawo)
        if abs(error_x) > DEADZONE_X:
            delta_x = (Kp * error_x) + (Kd * diff_x)
            eye_x += int(delta_x * smoothing)

        # Logika ruchu w osi Y (Góra-Dół) + Powieki
        if abs(error_y) > DEADZONE_Y:
            delta_y = (Kp * error_y) + (Kd * diff_y)
            move_y = int(delta_y * smoothing)

            eye_y -= move_y
            lid_1 -= move_y
            lid_2 += move_y
            lid_3 -= move_y
            lid_4 += move_y

        # zapisanie błędu do kolejnej iteracji
        prev_error_x = error_x
        prev_error_y = error_y

        # ograniczenia
        eye_x = max(tracking_limits["LeftRight"][0], min(tracking_limits["LeftRight"][1], eye_x))
        eye_y = max(tracking_limits["UpDown"][0], min(tracking_limits["UpDown"][1], eye_y))
        lid_1 = lid_limit(lid_1, "TopLeft")
        lid_2 = lid_limit(lid_2, "TopRight")
        lid_3 = lid_limit(lid_3, "BotLeft")
        lid_4 = lid_limit(lid_4, "BotRight")

        # aktualizacja serw
        set_servo("LeftRight", eye_x)
        set_servo("UpDown", eye_y)
        set_servo("TopLeft", lid_1)
        set_servo("BotRight", lid_4)
        set_servo("TopRight", lid_2)
        set_servo("BotLeft", lid_3)

        # wizualizacja
        cv2.rectangle(frame, (int(bbox.xmin * w), int(bbox.ymin * h)),
                      (int((bbox.xmin + bbox.width) * w), int((bbox.ymin + bbox.height) * h)),
                      (0, 255, 0), 2)
        cv2.circle(frame, (face_x, face_y), 5, (0, 0, 255), -1)

    else:
        # obliczanie błedu wzgledem srodka
        error_x = 90 - eye_x
        error_y = 65 - eye_y

        diff_x = error_x - prev_error_x
        diff_y = error_y - prev_error_y

        eye_x += (Kp * error_x) + (Kd * diff_x)
        eye_y += (Kp * error_y) + (Kd * diff_y)

        # powrót powiek do neutralnych pozycji
        lid_1 += (40 - lid_1) * Kp
        lid_2 += (140 - lid_2) * Kp
        lid_3 += (120 - lid_3) * Kp
        lid_4 += (60 - lid_4) * Kp


        prev_error_x = error_x
        prev_error_y = error_y

        # wysłanie wartości do serw
        set_servo("LeftRight", int(eye_x))
        set_servo("UpDown", int(eye_y))
        set_servo("TopLeft", int(lid_1))
        set_servo("TopRight", int(lid_2))
        set_servo("BotLeft", int(lid_3))
        set_servo("BotRight", int(lid_4))

    # mruganie co kilka sekund
    if time.time() - last_blink > next_blink:
        blink()
        last_blink = time.time()
        next_blink = random.uniform(4, 8)

    current_time = time.time()

    # ruch głowy
    if eye_x < EYE_LEFT_THRESHOLD:
        if current_neck_value < NECK_LEFT:
            current_neck_value += STEP_SIZE_UP
        else:
            current_neck_value = NECK_LEFT

    elif eye_x > EYE_RIGHT_THRESHOLD:

        if current_neck_value > NECK_RIGHT:
            current_neck_value -= STEP_SIZE_UP
        else:
            current_neck_value = NECK_RIGHT

    else:
        if current_neck_value > NECK_STOP:
            current_neck_value -= STEP_SIZE_DOWN
        elif current_neck_value < NECK_STOP:
            current_neck_value += STEP_SIZE_DOWN

        if abs(current_neck_value - NECK_STOP) < STEP_SIZE_DOWN:
            current_neck_value = NECK_STOP

    pi.set_servo_pulsewidth(NECK_PIN, current_neck_value)

    # wyświetlanie obrazu do testów
    cv2.imshow("Animatronic Eyes", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):  # wyjście po wciśnięciu 'q'
        break

cap.release()
cv2.destroyAllWindows()
calibrate()