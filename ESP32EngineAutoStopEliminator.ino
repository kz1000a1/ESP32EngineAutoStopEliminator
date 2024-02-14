#include "driver/twai.h"
#include "subaru_levorg_vnx.h"

// Pins used to connect to CAN bus transceiver:
#define RX_PIN GPIO_NUM_21
#define TX_PIN GPIO_NUM_20

#define POLLING_RATE_MS 1000
static bool driver_installed = false;

enum debug_mode DebugMode = DEBUG;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  //Look in the api-reference for other speed sets.
  // twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_filter_config_t f_config = { .acceptance_code = CAN_ID_CCU << 21, .acceptance_mask = ((CAN_ID_CCU << 21) ^ (CAN_ID_TCU << 21)) | 0x1fffff, .single_filter = true };

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("Driver installed");
  } else {
    Serial.println("Failed to install driver");
    return;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.println("Driver started");
  } else {
    Serial.println("Failed to start driver");
    return;
  }

  // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    Serial.println("CAN Alerts reconfigured");
  } else {
    Serial.println("Failed to reconfigure alerts");
    return;
  }

  // TWAI driver is now successfully installed and started
  driver_installed = true;
}


void loop() {
  twai_message_t rx_frame;
  twai_message_t tx_frame;
  tx_frame.identifier = CAN_ID_CCU;  //tx_frame.identifier;
  tx_frame.data_length_code = 8;
  // tx_frame.rtr = 0;
  // tx_frame.extended = false;

  uint32_t CurrentTime;

  static enum cu_status TcuStatus = ENGINE_STOP;
  static enum cu_status CcuStatus = ENGINE_STOP;
  static enum status Status = PROCESSING;
  static uint16_t PreviousCanId = CAN_ID_CCU;
  static uint8_t Retry = 0;

  if (!driver_installed) {
    // Driver not installed
    delay(1000);
    return;
  }
  // Check if alert happened
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
  twai_status_info_t twaistatus;
  twai_get_status_info(&twaistatus);

  // Handle alerts
  if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
    Serial.println("Alert: TWAI controller has become error passive.");
  }
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
    Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
    Serial.printf("Bus error count: %d\n", twaistatus.bus_error_count);
  }
  if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
    Serial.println("Alert: The RX queue is full causing a received frame to be lost.");
    Serial.printf("RX buffered: %d\t", twaistatus.msgs_to_rx);
    Serial.printf("RX missed: %d\t", twaistatus.rx_missed_count);
    Serial.printf("RX overrun %d\n", twaistatus.rx_overrun_count);
  }

  // Check if message is received
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    // One or more messages received. Handle all.
    if (twai_receive(&rx_frame, 0) == ESP_OK) {
      if (DebugMode != NORMAL) {

        CurrentTime = millis();

        // Output all received message(s) to CDC port as candump -L
        Serial.printf("(%d.%03d000) can0 %03X#%02X%02X%02X%02X%02X%02X%02X%02X\n",
                      CurrentTime / 1000,
                      CurrentTime % 1000,
                      rx_frame.identifier,
                      rx_frame.data[0],
                      rx_frame.data[1],
                      rx_frame.data[2],
                      rx_frame.data[3],
                      rx_frame.data[4],
                      rx_frame.data[5],
                      rx_frame.data[6],
                      rx_frame.data[7]);

        switch (rx_frame.identifier) {
          case CAN_ID_TCU:
            if ((rx_frame.data[2] & 0x08) != 0x08) {
              TcuStatus = NOT_READY;
            } else if (rx_frame.data[4] == 0xc0) {
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
            PreviousCanId = rx_frame.identifier;
            break;

          case CAN_ID_CCU:
            if (PreviousCanId == CAN_ID_CCU) {  // TCU don't transmit message
              CcuStatus = ENGINE_STOP;
              TcuStatus = ENGINE_STOP;
              Status = PROCESSING;
              Retry = 0;
            } else if (rx_frame.data[6] & 0x40) {
              if (DebugMode == DEBUG) {
                // Output Warning message
                Serial.printf("# Warning: Eliminate engine auto stop cancelled.\n");
              }
              Status = CANCELLED;
            } else if (Status == PROCESSING) {
              if (rx_frame.data[6] & 0x02) {
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
                  if ((rx_frame.data[1] & 0x0f) == 0x0f) {
                    tx_frame.data[1] = rx_frame.data[1] & 0xf0;
                  } else {
                    tx_frame.data[1] = rx_frame.data[1] + 0x01;
                  }
                  tx_frame.data[2] = rx_frame.data[2];
                  tx_frame.data[3] = rx_frame.data[3];
                  tx_frame.data[4] = rx_frame.data[4];
                  tx_frame.data[5] = rx_frame.data[5];
                  tx_frame.data[6] = rx_frame.data[6] | 0x40;  // Eliminate engine auto stop bit on
                  tx_frame.data[7] = rx_frame.data[7];
                  // Calculate checksum
                  tx_frame.data[0] = (tx_frame.data[1] + tx_frame.data[2] + tx_frame.data[3] + tx_frame.data[4] + tx_frame.data[5] + tx_frame.data[6] + tx_frame.data[7]) % SUM_CHECK_DIVIDER;

                  delay(50);  // 50ms delay like real CCU
                              // CAN0.sendFrame(tx_frame);
                  // Queue message for transmission
                  if (twai_transmit(&tx_frame, pdMS_TO_TICKS(1000)) == ESP_OK) {
                    printf("Message queued for transmission\n");
                  } else {
                    printf("Failed to queue message for transmission\n");
                  }
                  if (DebugMode == DEBUG) {
                    CurrentTime = millis();
                    // Output all transmitted message(s) to CDC port as candump -L
                    Serial.printf("# (%d.%03d000) can0 %03X#%02X%02X%02X%02X%02X%02X%02X%02X\n",
                                  CurrentTime / 1000,
                                  CurrentTime % 1000,
                                  tx_frame.identifier,
                                  tx_frame.data[0],
                                  tx_frame.data[1],
                                  tx_frame.data[2],
                                  tx_frame.data[3],
                                  tx_frame.data[4],
                                  tx_frame.data[5],
                                  tx_frame.data[6],
                                  tx_frame.data[7]);
                  }

                  // Discard message(s) that received during delay()
                  // while (CAN0.read(rx_frame)){}
                  while (twai_receive(&rx_frame, 0) == ESP_OK) {}

                  Retry++;
                }
              } else {  // Unexpected case
                if (DebugMode == DEBUG) {
                  // Output Warning message
                  Serial.printf("# Warning: Unexpected case (CCU=%d TCU=%d).\n", CcuStatus, TcuStatus);
                }
              }
            }

            PreviousCanId = rx_frame.identifier;
            break;

          default:  // Unexpected can id
            if (DebugMode == DEBUG) {

              // Output Warning message
              Serial.printf("# Warning: Unexpected can id (0x%03x).\n", rx_frame.identifier);
            }

            break;
        }
      }
    }
  }
}
