#include <Arduino.h>

/*
 * Включаем универсальный декодер для pulse distance/width протоколов.
 * Это ДОЛЖНО быть сделано до #include <IRremote.hpp>
 */
#define DECODE_DISTANCE_WIDTH   // Универсальный декодер для pulse distance width протоколов

#include <IRremote.hpp>

// если мощности не хватит с пинов, то можно через транзистор подключить:
// GPIO4 ──[1кОм]── База 2N2222 (NPN)
//                 Коллектор ──[100 Ом]── IR LED ── VCC (3.3V или 5V)
//                 Эмиттер ─── GND

// Параметры выстрела по умолчанию
#define DEFAULT_TEAM    1
#define DEFAULT_DAMAGE  1
#define DEFAULT_ID      1

// Тайминги MilesTag II Data Protocol (Pulse Width)
// Частота 38 кГц
// Тайминги взяты из реального замера сигнала тагера:
//   Header: 2400, 600
//   Bit 1:  1250, 550
//   Bit 0:   650, 550
#define MILES_TAG_FREQ_KHZ      38
#define MILES_TAG_HEADER_MARK   2400
#define MILES_TAG_HEADER_SPACE  600
#define MILES_TAG_ONE_MARK      1250
#define MILES_TAG_ONE_SPACE     550
#define MILES_TAG_ZERO_MARK     650
#define MILES_TAG_ZERO_SPACE    550

// Пины настраиваются через build_flags в platformio.ini
// По умолчанию: ESP-01 — MY_IR_SEND_PIN=3 (GPIO3/RX), MY_IR_RECV_PIN=2 (GPIO2)
//               ESP-12 — MY_IR_SEND_PIN=4 (GPIO4/D2), MY_IR_RECV_PIN=5 (GPIO5/D1)
#ifndef MY_IR_SEND_PIN
#define MY_IR_SEND_PIN  3   // GPIO3 (RX) для ESP-01
#endif
#ifndef MY_IR_RECV_PIN
#define MY_IR_RECV_PIN  2   // GPIO2 для ESP-01
#endif

// Маркер конца пакета для команд и сообщений
#define MILES_TAG_END_MARKER   0xE8

// Коды команд MilesTag II
#define CMD_DETONATE    0x0B   // Взорвать игрока
#define CMD_KILL        0x00   // Убийство администратором
#define CMD_PAUSE       0x01   // Поставить/снять паузу
#define CMD_START_GAME  0x02   // Начать игру
#define CMD_RESPAWN     0x04   // Респаун
#define CMD_NEW_GAME    0x05   // Новая игра (немедленно)
#define CMD_FILL_CLIP   0x06   // Заполнить магазин
#define CMD_END_GAME    0x07   // Закончить игру
#define CMD_INIT_PLAYER 0x0A   // Инициализировать игрока
#define CMD_NEW_GAME_RDY 0x0C  // Новая игра (готовность)
#define CMD_RESTORE_LIFE 0x0D  // Восстановить жизнь
#define CMD_RESTORE_ARMOR 0x0F // Восстановить броню
#define CMD_RESET_SCORE 0x14   // Сбросить очки
#define CMD_CHECK_SENSORS 0x15 // Проверить датчики
#define CMD_IMMOBILIZE  0x16   // Обездвижить игрока
#define CMD_UNLOAD      0x17   // Разрядить оружие

// Маркеры пакетов MilesTag II
#define MT_SHOOT_MARKER    0   // Бит 13 = 0 для выстрела
#define MT_COMMAND_MARKER  0x83
#define MT_MESSAGE_MARKER  0xE8
#define MT_SYSTEM_MARKER   0x87

// Структура с таймингами протокола MilesTag II для библиотеки IRremote
static const DistanceWidthTimingInfoStruct sMilesTagTiming = {
    MILES_TAG_HEADER_MARK,   // HeaderMarkMicros
    MILES_TAG_HEADER_SPACE,  // HeaderSpaceMicros
    MILES_TAG_ONE_MARK,      // OneMarkMicros
    MILES_TAG_ONE_SPACE,     // OneSpaceMicros
    MILES_TAG_ZERO_MARK,     // ZeroMarkMicros
    MILES_TAG_ZERO_SPACE     // ZeroSpaceMicros
};

// Флаг, указывающий, что идёт отправка (чтобы игнорировать собственный сигнал)
volatile bool isSending = false;
// Время последней отправки (для игнорирования отражённого сигнала)
unsigned long lastSendTime = 0;

#define SHOOT_WINDOW 500
#define DETONATE_DELAY 500

// Состояние игры
uint8_t lives = 0;              // 0 = игра не начата, >0 = текущее количество жизней
unsigned long lastHitTime = 0;  // Время последнего попадания (для защиты 500 мс)
bool detonatePending = false;   // Флаг: нужно отправить Detonate через 500 мс
unsigned long detonateTime = 0; // Время, когда нужно отправить Detonate

/**
 * Преобразует значение повреждения (1..100) в 4-битный код MilesTag II.
 * Таблица соответствия из спецификации протокола.
 */
uint8_t damageToCode(uint8_t damage) {
  if (damage <= 1)   return 0b0000;
  if (damage <= 2)   return 0b0001;
  if (damage <= 4)   return 0b0010;
  if (damage <= 5)   return 0b0011;
  if (damage <= 7)   return 0b0100;
  if (damage <= 10)  return 0b0101;
  if (damage <= 15)  return 0b0110;
  if (damage <= 17)  return 0b0111;
  if (damage <= 20)  return 0b1000;
  if (damage <= 25)  return 0b1001;
  if (damage <= 30)  return 0b1010;
  if (damage <= 35)  return 0b1011;
  if (damage <= 40)  return 0b1100;
  if (damage <= 50)  return 0b1101;
  if (damage <= 75)  return 0b1110;
  return 0b1111;  // 100
}

/**
 * Преобразует 4-битный код повреждения в значение (1..100).
 */
uint8_t codeToDamage(uint8_t code) {
  switch (code & 0xF) {
    case 0b0000: return 1;
    case 0b0001: return 2;
    case 0b0010: return 4;
    case 0b0011: return 5;
    case 0b0100: return 7;
    case 0b0101: return 10;
    case 0b0110: return 15;
    case 0b0111: return 17;
    case 0b1000: return 20;
    case 0b1001: return 25;
    case 0b1010: return 30;
    case 0b1011: return 35;
    case 0b1100: return 40;
    case 0b1101: return 50;
    case 0b1110: return 75;
    case 0b1111: return 100;
    default:     return 0;
  }
}

/**
 * Декодирует пакет MilesTag II.
 * Данные уже в MSB-first формате (как в спецификации).
 */
void decodePacket(uint32_t data, uint16_t bits) {
  if (bits != 14 && bits != 24) return;

  if (bits == 24) {
    uint8_t byte1 = (data >> 16) & 0xFF;  // Старший байт
    uint8_t byte2 = (data >> 8) & 0xFF;   // Средний байт
    uint8_t byte3 = data & 0xFF;          // Младший байт

    Serial.println("=== Command ===");
    Serial.print("Bytes: ");
    Serial.print(byte1, HEX);
    Serial.print(" ");
    Serial.print(byte2, HEX);
    Serial.print(" ");
    Serial.println(byte3, HEX);

    if (byte1 == MT_COMMAND_MARKER && byte3 == MILES_TAG_END_MARKER) {
      // Пакет команды: 0x83 + Command + 0xE8
      Serial.print("Command code: 0x");
      Serial.println(byte2, HEX);

      switch (byte2) {
        case CMD_DETONATE:
          Serial.println("-> Detonate (взорвать игрока)");
          break;
        case CMD_KILL:
          Serial.println("-> Admin kill");
          break;
        case CMD_PAUSE:
          Serial.println("-> Pause/Unpause");
          break;
        case CMD_START_GAME:
          Serial.println("-> Start game");
          lives = 100;
          lastHitTime = 0;
          detonatePending = false;
          Serial.print("Lives set to: ");
          Serial.println(lives);
          break;
        case CMD_RESPAWN:
          Serial.println("-> Respawn");
          break;
        case CMD_NEW_GAME:
          Serial.println("-> New game (immediate)");
          break;
        case CMD_FILL_CLIP:
          Serial.println("-> Fill clip");
          break;
        case CMD_END_GAME:
          Serial.println("-> End game");
          break;
        case CMD_INIT_PLAYER:
          Serial.println("-> Init player");
          break;
        case CMD_NEW_GAME_RDY:
          Serial.println("-> New game (ready)");
          break;
        case CMD_RESTORE_LIFE:
          Serial.println("-> Restore life");
          break;
        case CMD_RESTORE_ARMOR:
          Serial.println("-> Restore armor");
          break;
        case CMD_RESET_SCORE:
          Serial.println("-> Reset score");
          break;
        case CMD_CHECK_SENSORS:
          Serial.println("-> Check sensors");
          break;
        case CMD_IMMOBILIZE:
          Serial.println("-> Immobilize");
          break;
        case CMD_UNLOAD:
          Serial.println("-> Unload weapon");
          break;
        default:
          Serial.println("-> Unknown command");
          break;
      }
    } else if (byte1 == MT_MESSAGE_MARKER || byte3 == MILES_TAG_END_MARKER) {
      Serial.println("-> Message packet");
    } else if (byte1 == MT_SYSTEM_MARKER) {
      Serial.println("-> System data packet");
    } else {
      Serial.println("-> Unknown packet type");
    }
  } else {
    // 14 бит — пакет выстрела
    // Формат: 0 + ID(7) + Team(2) + DamageCode(4)
    uint8_t id = (data >> 6) & 0x7F;
    uint8_t team = (data >> 4) & 0x3;
    uint8_t dmgCode = data & 0xF;
    uint8_t dmgValue = codeToDamage(dmgCode);

    Serial.println("=== Shoot ===");
    Serial.print("ID: ");
    Serial.println(id);
    Serial.print("Team: ");
    Serial.println(team);
    Serial.print("Damage code: ");
    Serial.println(dmgCode, BIN);
    Serial.print("Damage value: ");
    Serial.println(dmgValue);

    // Обработка попадания
    if (lives > 0) {
      unsigned long now = millis();
      if (now - lastHitTime >= SHOOT_WINDOW) {
        lastHitTime = now;
        lives = (dmgValue >= lives) ? 0 : lives - dmgValue;
        Serial.print("Lives remaining: ");
        Serial.println(lives);

        if (lives <= 0) {
          Serial.println("Player destroyed! Detonate pending in DETONATE_DELAYms...");
          detonatePending = true;
          detonateTime = now + DETONATE_DELAY;
        }
      } else {
        Serial.println("Hit ignored (SHOOT_WINDOWms protection)");
      }
    } else {
      Serial.println("Shoot ignored (game not started)");
    }
  }
}

/**
 * Формирует 14-битный пакет выстрела MilesTag II.
 * Формат (MSB first): 0 + ID(7) + Team(2) + DamageCode(4)
 */
uint16_t buildShootPacket(uint8_t id, uint8_t team, uint8_t damage) {
  uint8_t dmgCode = damageToCode(damage);
  return ((uint16_t)(id & 0x7F) << 6) | ((team & 0x3) << 4) | (dmgCode & 0xF);
}

/**
 * Отправляет произвольный пакет данных по протоколу MilesTag II (Pulse Width, MSB first).
 * Использует библиотечную функцию IrSender.sendPulseDistanceWidth().
 *
 * @param data   данные для отправки
 * @param bits   количество бит (14 для выстрела, 24 для команды)
 */
void sendPacket(uint32_t data, uint8_t bits) {
  isSending = true;

  // Отправляем через библиотечную функцию с флагом PROTOCOL_IS_MSB_FIRST
  // Параметры: частота, тайминги header/mark/space для 0/1, данные, биты, флаги, период повтора, кол-во повторов
  IrSender.sendPulseDistanceWidth(
      MILES_TAG_FREQ_KHZ,
      MILES_TAG_HEADER_MARK, MILES_TAG_HEADER_SPACE,
      MILES_TAG_ONE_MARK, MILES_TAG_ONE_SPACE,
      MILES_TAG_ZERO_MARK, MILES_TAG_ZERO_SPACE,
      data,
      bits,
      PROTOCOL_IS_MSB_FIRST,  // MilesTag использует MSB first
      0,                       // RepeatPeriodMillis = 0 (без повторов)
      0                        // NumberOfRepeats = 0
  );

  isSending = false;
  lastSendTime = millis();
}

/**
 * Отправляет пакет выстрела.
 * Протокол MilesTag II: 14 бит, MSB first.
 */
void sendShoot(uint8_t id, uint8_t team, uint8_t damage) {
  uint16_t packet = buildShootPacket(id, team, damage);

  Serial.println("--- Sending shoot ---");
  Serial.print("ID: ");
  Serial.println(id);
  Serial.print("Team: ");
  Serial.println(team);
  Serial.print("Damage: ");
  Serial.println(damage);
  Serial.print("Packet: 0x");
  Serial.println(packet, HEX);
  Serial.print("Packet (bin): ");
  Serial.println(packet, BIN);

  sendPacket(packet, 14);

  Serial.println("--- Shoot sent ---");
}

/**
 * Отправляет 24-битную команду MilesTag II.
 * Формат: 0x83 + Command + 0xE8
 */
void sendCommand(uint8_t command) {
  uint32_t packet = ((uint32_t)MT_COMMAND_MARKER << 16) | ((uint32_t)command << 8) | MILES_TAG_END_MARKER;

  Serial.println("--- Sending command ---");
  Serial.print("Command: 0x");
  Serial.println(command, HEX);
  Serial.print("Packet: 0x");
  Serial.println(packet, HEX);

  sendPacket(packet, 24);

  Serial.println("--- Command sent ---");
}

/**
 * Отправляет команду "взорвать игрока" (Detonate).
 */
void sendDetonate() {
  sendCommand(CMD_DETONATE);
}

void setup() {
  Serial.begin(9600);
  Serial.println("setup");

  // Инициализация приёмника
  IrReceiver.begin(MY_IR_RECV_PIN, ENABLE_LED_FEEDBACK);
  Serial.print("IR receiver started on pin ");
  Serial.println(MY_IR_RECV_PIN);

  // Инициализация передатчика
  // Для ESP8266 без IR_SEND_PIN нужно передать пин явно
  IrSender.begin(MY_IR_SEND_PIN, false, 0);  // false = без LED feedback
  Serial.print("IR sender started on pin ");
  Serial.println(MY_IR_SEND_PIN);
}

void loop() {
  // Отправка Detonate через 500 мс после смерти игрока
  if (detonatePending && millis() - detonateTime >= DETONATE_DELAY) {
    detonatePending = false;
    sendDetonate();
    IrReceiver.restartAfterSend();
  }

  // Приём IR-сигналов
  if (IrReceiver.decode()) {
    // Игнорируем собственный сигнал (во время отправки и 100 мс после)
    if (!isSending && (millis() - lastSendTime > 100)) {
      // Протокол PULSE_WIDTH или PULSE_DISTANCE — то, что нужно для MilesTag
      if (IrReceiver.decodedIRData.protocol == PULSE_WIDTH ||
          IrReceiver.decodedIRData.protocol == PULSE_DISTANCE) {

        uint16_t bits = IrReceiver.decodedIRData.numberOfBits;
        uint32_t data = IrReceiver.decodedIRData.decodedRawData;

        // Маскируем только значимые биты
        if (bits < 32) {
          data &= ((1UL << bits) - 1);
        }

        // Библиотека IRremote для PULSE_WIDTH/PULSE_DISTANCE сохраняет биты LSB-first
        // (первый принятый бит = LSB). MilesTag II использует MSB-first,
        // поэтому разворачиваем биты.
        uint32_t dataRev = 0;
        for (uint8_t i = 0; i < bits; i++) {
          if (data & (1UL << i)) {
            dataRev |= (1UL << (bits - 1 - i));
          }
        }

        decodePacket(dataRev, bits);
      }
    }

    IrReceiver.resume();
  }
}
