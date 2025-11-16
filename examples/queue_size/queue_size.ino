#include <RP2040PIO_CAN.h>

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.print("sizeof(CanMsg) = ");
  Serial.println(sizeof(CanMsg));
  const size_t queue_size = 8;
  Serial.print("estimated bytes for queue: ");
  Serial.println(sizeof(CanMsg) * queue_size);
  Serial.print("estimated KB: ");
  Serial.println((sizeof(CanMsg) * queue_size) / 1024.0, 2);
}
void loop() {}
