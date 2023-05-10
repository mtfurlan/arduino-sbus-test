#include <Arduino.h>
#include <sbus.h>
// https://www.waveshare.com/img/devkit/accBoard/NodeMCU-32S/NodeMCU-32S-details-3.jpg



// running an accst v1 mode

#define SBUS_RX 18
#define SBUS_TX 19


/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial2, SBUS_RX, SBUS_TX, false);
//bfs::SbusTx sbus_tx(&Serial2, SBUS_RX, SBUS_TX, false);
/* SBUS data */
bfs::SbusData data;

void setup() {
  /* Serial to display data */
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("starting...");



  Serial2.begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin, bool invert)
  /* Begin the SBUS communication */
  sbus_rx.Begin();
  //sbus_tx.Begin();
}

void loop () {
  if (sbus_rx.Read()) {
    /* Grab the received data */
    data = sbus_rx.data();
    /* Display the received data */
    for (int8_t i = 0; i < data.NUM_CH; i++) {
      Serial.print(data.ch[i]);
      Serial.print("\t");
    }
    /* Display lost frames and failsafe data */
    Serial.print(data.lost_frame);
    Serial.print("\t");
    Serial.println(data.failsafe);
    /* Set the SBUS TX data to the received data */
    //sbus_tx.data(data);
    /* Write the data to the servos */
    //sbus_tx.Write();
  }
}
