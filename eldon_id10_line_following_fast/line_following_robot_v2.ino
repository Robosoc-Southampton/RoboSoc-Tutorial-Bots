#define KP 24
#define KD 2.9
#define BASE 120

// Swag values (KP, KD, BASE)
// 1: 12, 1.5, 75
// 2: 15, 1.3, 85
// 3: 20, 2.5, 105
// 4: 24, 2.9, 120 (Only just works)

// Values slow: 15 10 70

const int LEFT_BIAS = 0;

bool parseLong(const char *input, long &result) {
  char *end;
  result = strtol(input, &end, 10);

  return end != input && *end == '\0';
}

void setMotors(int left, int right) {
  if (left < 0) {
    digitalWrite(3, 0);
    digitalWrite(2, 1);
  } else if (left == 0) {
    digitalWrite(3, 0);
    digitalWrite(2, 0);
  } else {
    digitalWrite(3, 1);
    digitalWrite(2, 0);
  }

  if (right < 0) {
    digitalWrite(4, 0);
    digitalWrite(6, 1);
  } else if (right == 0) {
    digitalWrite(4, 0);
    digitalWrite(6, 0);
  } else {
    digitalWrite(4, 1);
    digitalWrite(6, 0);
  }

  analogWrite(9, min(abs(left) + LEFT_BIAS, 255));
  analogWrite(10, min(abs(right) - LEFT_BIAS, 255));
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(9, OUTPUT);   //ENA
  pinMode(2, OUTPUT);   //IN1
  pinMode(3, OUTPUT);   //IN2
  pinMode(4, OUTPUT);   //IN3
  pinMode(6, OUTPUT);   //IN4
  pinMode(10, OUTPUT);  //ENB

  pinMode(8, INPUT);  // Left sensor (1=black)
  pinMode(7, INPUT);  // Right sensor (1=black)

  setMotors(0, 0);
}



// call every control tick; level = -2..+2, now in microseconds
static int   lastLevel = 0, lastDir = 0;
static uint32_t tEnter = 0;
static float levelRate = 0, rateOut = 0;   // levels per second

float update_rate(int level, uint32_t now) {
    if (level != lastLevel) {
        float dwell = (now - tEnter) * 1e-6f;
        int   dir   = (level > lastLevel) ? 1 : -1;
        if (dir == lastDir || lastDir == 0)
            levelRate = (level - lastLevel) / dwell;   // full level crossed
        else
            levelRate = dir * fabsf(rateOut);          // reversal: flip sign
        lastDir = dir; lastLevel = level; tEnter = now;
    }
    float tSince = fmaxf((now - tEnter) * 1e-6f, 1e-3f);
    float cap = 1.0f / tSince;
    rateOut = (fabsf(levelRate) > cap) ? copysignf(cap, levelRate) : levelRate; // Ensure "levelRate" is less than or equal to "cap"
    return rateOut;
}

// output = Kp * level + Kd * update_rate(level, micros());






char message[20];

int error = 0, prevError = 0;     // prevError = last *different* error
int lastSide = -1;

void loop() {
  bool L = digitalRead(8), R = digitalRead(7);
  int e;
  if (L && R)      e = 0;
  else if (L)      { e = -1; lastSide = -1; }
  else if (R)      { e = +1; lastSide = +1; }
  else             e = 2 * lastSide;          // lost: assume it's past the last side seen

  unsigned long now = micros();
  if (e != error) { prevError = error; error = e; }

  float d = update_rate(e, now);
  // Serial.print("New rate: "); Serial.print(d); Serial.println("");

  float steer = KP * error + KD * d;
  setMotors(static_cast<int>(BASE + steer), static_cast<int>(BASE - steer));       // clamp to motor limits

}



