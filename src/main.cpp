#include <Arduino.h>
#include <sbus.h>
// https://www.waveshare.com/img/devkit/accBoard/NodeMCU-32S/NodeMCU-32S-details-3.jpg



// running an accst v1 mode

#define SBUS_RX 19
#define SBUS_TX 18 // unused


/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial2, SBUS_RX, SBUS_TX, true);

/* SBUS data */
bfs::SbusData data;

void setup() {
  /* Serial to display data */
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("starting...");



  /* Begin the SBUS communication */
  sbus_rx.Begin();
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
  }
}
