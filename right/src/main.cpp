#include <HX711.h>

#define HX_DOUT 4
#define HX_SCK  21

HX711 scale;

long sum0 = 0, cnt0 = 0, min0 = LONG_MAX, max0 = LONG_MIN;
long sum1 = 0, cnt1 = 0, min1 = LONG_MAX, max1 = LONG_MIN;

unsigned long tStart;
int phase = 0; // 0 = tare wait, 1 = no weight, 2 = with weight, 3 = done

void setup() {
  Serial.begin(115200);
  delay(300);

  scale.begin(HX_DOUT, HX_SCK);
  pinMode(HX_DOUT, INPUT_PULLUP);

  Serial.println("STEP 0: REMOVE ALL WEIGHT");
  delay(3000);

  if (!scale.wait_ready_timeout(3000)) {
    Serial.println("HX711 not ready");
    while (1);
  }

  scale.tare();
  Serial.println("Tare done");

  Serial.println("STEP 1: 20s WITHOUT WEIGHT");
  tStart = millis();
  phase = 1;
}

void loop() {
  if (phase == 1) {
    if (millis() - tStart <= 20000) {
      if (scale.wait_ready_timeout(1000)) {
        long v = scale.read();
        sum0 += v; cnt0++;
        if (v < min0) min0 = v;
        if (v > max0) max0 = v;
      }
    } else {
      Serial.println("STEP 2: PLACE 1.083 kg NOW (center) – measuring 20s");
      tStart = millis();
      phase = 2;
    }
  }

  else if (phase == 2) {
    if (millis() - tStart <= 20000) {
      if (scale.wait_ready_timeout(1000)) {
        long v = scale.read();
        sum1 += v; cnt1++;
        if (v < min1) min1 = v;
        if (v > max1) max1 = v;
      }
    } else {
      phase = 3;

      Serial.println("\n===== RESULTS =====");

      long avg0 = cnt0 ? sum0 / cnt0 : 0;
      long avg1 = cnt1 ? sum1 / cnt1 : 0;

      Serial.println("NO WEIGHT:");
      Serial.print("  samples: "); Serial.println(cnt0);
      Serial.print("  avg raw: "); Serial.println(avg0);
      Serial.print("  min raw: "); Serial.println(min0);
      Serial.print("  max raw: "); Serial.println(max0);
      Serial.print("  noise p-p: "); Serial.println(max0 - min0);

      Serial.println("\nWITH 1.083 kg:");
      Serial.print("  samples: "); Serial.println(cnt1);
      Serial.print("  avg raw: "); Serial.println(avg1);
      Serial.print("  min raw: "); Serial.println(min1);
      Serial.print("  max raw: "); Serial.println(max1);
      Serial.print("  noise p-p: "); Serial.println(max1 - min1);

      long delta = avg1 - avg0;
      Serial.println("\nDELTA (avg1 - avg0):");
      Serial.println(delta);

      float factor = delta / 1.083f;
      Serial.print("CALC calibration factor (delta / 1.083): ");
      Serial.println(factor, 3);

      while (1); // stop
    }
  }
}