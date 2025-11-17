#include <RP2040PIO_CAN.h>  //ボードに応じてCANライブラリを変更
#include <CANDemux.h>

//CANライブラリよりも下で呼び出す api/HardwareCAN.hが無いって言われる
#include <C6x0.h>  //https://github.com/tutrc-freshman/TUTRC_ArduinoLib.git
#include "DM.h"    // 作りかけ

const uint32_t CAN_TX_PIN = 0;
const uint32_t CAN_RX_PIN = 1;

const uint32_t dm_masterID = 0;
const uint32_t dm_slaveID_A = 9;
const uint32_t dm_slaveID_B = 10;

CANDemux canDemux(&CAN);
VirtualCAN c6x0_vcan = canDemux.createClientWithRange(0x201, 8, 1);
VirtualCAN dm_vcan = canDemux.createClientWithIds({ dm_masterID }, 2);

C6x0 c6x0;
DMManager dmManager(dm_masterID);
DMMotor dmMotorA(&dmManager, dm_slaveID_A, DM_ControlMode::DM_CM_MIT);
DMMotor dmMotorB(&dmManager, dm_slaveID_B, DM_ControlMode::DM_CM_MIT);

unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL = 200;  // ms

void setup() {
  Serial.begin(115200);
  delay(200);

  // CAN 初期化
  CAN.setTX(CAN_TX_PIN);
  CAN.setRX(CAN_RX_PIN);
  CAN.begin(CanBitRate::BR_1000k);

  // VirtualCAN を割り当て
  c6x0.setCAN(&c6x0_vcan);
  dmManager.setCAN(&dm_vcan);

  // モーター初期化（例）
  dmMotorA.initialize();
  dmMotorB.initialize();
  dmMotorA.setZeroPoint();
  dmMotorB.setZeroPoint();
  dmMotorA.enable();
  dmMotorB.enable();
}

void loop() {
  // CANクライアントの処理を定期実行
  c6x0.update();
  dmManager.update();

  float kp = 100;
  float rps = c6x0.getRpm(C610_ID_1) / 60.0f;
  float rps_ref = 10;
  float current_ref = kp * (rps_ref - rps);

  c6x0.setCurrent(C610_ID_1, current_ref);
  c6x0.transmit();  // CANバスに送信される

  dmMotorA.sendMIT(0.0f, 0.5f, 0.0f, 1.0f, 0.0f);
  dmMotorB.sendMIT(0.0f, 0.5f, 0.0f, 1.0f, 0.0f);

  unsigned long now = millis();
  if (now - lastPrint >= PRINT_INTERVAL) {
    lastPrint = now;

    // C6x0 の状態表示（ID 1 を例に）
    float angle = c6x0.getPosition(C610_ID_1);
    float rpm = c6x0.getRpm(C610_ID_1);
    Serial.print("C6x0 ID=0x");
    Serial.print(0x201 + C610_ID_1, HEX);
    Serial.print(" Angle=");
    Serial.print(angle);
    Serial.print(" deg, RPM=");
    Serial.print(rpm);

    // DM モーター状態表示
    Serial.print("  |  DM A status=");
    Serial.print((int)dmMotorA.getStatus());
    Serial.print(" pos_deg=");
    Serial.print(dmMotorA.getPositionDeg());
    Serial.print(" torque=");
    Serial.print(dmMotorA.getTorque());

    Serial.print("  |  DM B status=");
    Serial.print((int)dmMotorB.getStatus());
    Serial.print(" pos_deg=");
    Serial.print(dmMotorB.getPositionDeg());
    Serial.print(" torque=");
    Serial.print(dmMotorB.getTorque());

    Serial.println();
  }
}
