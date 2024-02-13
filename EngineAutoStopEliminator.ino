#include <esp32_can.h>
#include "subaru_levorg_vnx.h"

enum debug_mode DebugMode = DEBUG;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  CAN0.setCANPins(GPIO_NUM_21, GPIO_NUM_20);
  // CAN0.setDebuggingMode(true);
  CAN0.begin(500000);
  CAN0.watchFor(CAN_ID_CCU);
  CAN0.watchFor(CAN_ID_TCU);
}

void loop() {
  CAN_FRAME rx_frame;
  CAN_FRAME tx_frame;
  tx_frame.id = CAN_ID_CCU;  //tx_frame.id;
  tx_frame.length = 8;
  tx_frame.rtr = 0;
  tx_frame.extended = false;


  uint32_t CurrentTime;

  static enum cu_status TcuStatus = ENGINE_STOP;
  static enum cu_status CcuStatus = ENGINE_STOP;
  static enum status Status = PROCESSING;
  static uint16_t PreviousCanId = CAN_ID_CCU;
  static uint8_t Retry = 0;

  // If CAN message receive is pending, process the message
  if (CAN0.read(rx_frame)) {

    if (DebugMode != NORMAL) {

      CurrentTime = millis();

      // Output all received message(s) to CDC port as candump -L
      Serial.printf("(%d.%03d000) can0 %03X#%02X%02X%02X%02X%02X%02X%02X%02X\n",
                    CurrentTime / 1000,
                    CurrentTime % 1000,
                    rx_frame.id,
                    rx_frame.data.uint8[0],
                    rx_frame.data.uint8[1],
                    rx_frame.data.uint8[2],
                    rx_frame.data.uint8[3],
                    rx_frame.data.uint8[4],
                    rx_frame.data.uint8[5],
                    rx_frame.data.uint8[6],
                    rx_frame.data.uint8[7]);

      switch (rx_frame.id) {
        case CAN_ID_TCU:
          if ((rx_frame.data.uint8[2] & 0x08) != 0x08) {
            TcuStatus = NOT_READY;
          } else if (rx_frame.data.uint8[4] == 0xc0) {
            TcuStatus = IDLING_STOP_OFF;
            if (Retry != 0 && Status == PROCESSING) {
              if (DebugMode == DEBUG) {
                // Output Warning message
                Serial.printf("# Warning: Eliminate engine auto stop succeeded.\n");
              }
              Status = SUCCEEDED;
            }
          } else {
            TcuStatus = IDLING_STOP_ON;
            if (Status == SUCCEEDED) {
              if (DebugMode == DEBUG) {
                // Output Warning message
                Serial.printf("# Warning: Eliminate engine auto stop restarted.\n");
              }
              Status = PROCESSING;
              CcuStatus = NOT_READY;
              Retry = 0;
            }
          }
          PreviousCanId = rx_frame.id;
          break;

        case CAN_ID_CCU:
          if (PreviousCanId == CAN_ID_CCU) {  // TCU don't transmit message
            CcuStatus = ENGINE_STOP;
            TcuStatus = ENGINE_STOP;
            Status = PROCESSING;
            Retry = 0;
          } else if (rx_frame.data.uint8[6] & 0x40) {
            if (DebugMode == DEBUG) {
              // Output Warning message
              Serial.printf("# Warning: Eliminate engine auto stop cancelled.\n");
            }
            Status = CANCELLED;
          } else if (Status == PROCESSING) {
            if (rx_frame.data.uint8[6] & 0x02) {
              CcuStatus = NOT_READY;
            } else if (CcuStatus == NOT_READY || CcuStatus == ENGINE_STOP || TcuStatus == IDLING_STOP_OFF) {
              CcuStatus = READY;
            } else if (TcuStatus == IDLING_STOP_ON) {  // Transmit message for eliminate engine auto stop
              if (MAX_RETRY <= Retry) {                // Previous eliminate engine auto stop message failed
                if (DebugMode == DEBUG) {

                  // Output Warning message
                  Serial.printf("# Warning: Eliminate engine auto stop failed\n");
                }

                Status = FAILED;

              } else {

                // Increment counter
                if ((rx_frame.data.uint8[1] & 0x0f) == 0x0f) {
                  tx_frame.data.uint8[1] = rx_frame.data.uint8[1] & 0xf0;
                } else {
                  tx_frame.data.uint8[1] = rx_frame.data.uint8[1] + 0x01;
                }
                tx_frame.data.uint8[2] = rx_frame.data.uint8[2];
                tx_frame.data.uint8[3] = rx_frame.data.uint8[3];
                tx_frame.data.uint8[4] = rx_frame.data.uint8[4];
                tx_frame.data.uint8[5] = rx_frame.data.uint8[5];
                tx_frame.data.uint8[6] = rx_frame.data.uint8[6] | 0x40;  // Eliminate engine auto stop bit on
                tx_frame.data.uint8[7] = rx_frame.data.uint8[7];
                // Calculate checksum
                tx_frame.data.uint8[0] = (tx_frame.data.uint8[1] + tx_frame.data.uint8[2] + tx_frame.data.uint8[3] + tx_frame.data.uint8[4] + tx_frame.data.uint8[5] + tx_frame.data.uint8[6] + tx_frame.data.uint8[7]) % SUM_CHECK_DIVIDER;

                delay(50); // 50ms delay like real CCU
                CAN0.sendFrame(tx_frame);

                if (DebugMode == DEBUG) {
                  CurrentTime = millis();
                  // Output all transmitted message(s) to CDC port as candump -L
                  Serial.printf("# (%d.%03d000) can0 %03X#%02X%02X%02X%02X%02X%02X%02X%02X\n",
                                CurrentTime / 1000,
                                CurrentTime % 1000,
                                tx_frame.id,
                                tx_frame.data.uint8[0],
                                tx_frame.data.uint8[1],
                                tx_frame.data.uint8[2],
                                tx_frame.data.uint8[3],
                                tx_frame.data.uint8[4],
                                tx_frame.data.uint8[5],
                                tx_frame.data.uint8[6],
                                tx_frame.data.uint8[7]);
                }

                // Discard message(s) that received during delay()
                while (CAN0.read(rx_frame)){}

                Retry++;
              }
            } else {  // Unexpected case
              if (DebugMode == DEBUG) {
                // Output Warning message
                Serial.printf("# Warning: Unexpected case (CCU=%d TCU=%d).\n", CcuStatus, TcuStatus);
              }
            }
          }

          PreviousCanId = rx_frame.id;
          break;

        default:  // Unexpected can id
          if (DebugMode == DEBUG) {

            // Output Warning message
            Serial.printf("# Warning: Unexpected can id (0x%03x).\n", rx_frame.id);
          }

          break;
      }
    }
  }
}
