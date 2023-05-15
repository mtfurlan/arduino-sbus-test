#include <Arduino.h>
#include <sbus.h>
#include <FrskySP.h>
#include <Servo.h>
// https://www.waveshare.com/img/devkit/accBoard/NodeMCU-32S/NodeMCU-32S-details-3.jpg



// running an accst v1 mode

#define SBUS_RX 19
#define SBUS_TX 18 // unused

#define SPORT 17

#define SPORT_LED 32

#define LED_R 33
#define LED_G 25
#define LED_B 26
#define TEST_SW 27


Servo steering;
Servo drive;

#define MOTOR_PIN 16
#define STEERING_PIN 4
#define MOTOR_CHAN 0
#define STEERING_CHAN 1

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial2, SBUS_RX, SBUS_TX, true);


FrskySP FrskySP(&Serial1, SPORT);

/* SBUS data */
bfs::SbusData data;

void setup() {
    /* Serial to display data */
    Serial.begin(115200);
    while (!Serial) {}
    Serial.println("starting...");

    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);
    pinMode(TEST_SW, INPUT_PULLUP);


    steering.attach(STEERING_PIN, STEERING_CHAN);
    drive.attach(MOTOR_PIN, MOTOR_CHAN);

    /* Begin the SBUS communication */
    sbus_rx.Begin();

    FrskySP.ledSet(SPORT_LED);
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

        // channels are 172 - 1811
        // steering range is in degrees


        digitalWrite (LED_R, data.ch[0] > 500);
        digitalWrite (LED_G, data.ch[3] > 1000);
        digitalWrite (LED_B, data.ch[3] < 500);
        int degrees = map(data.ch[3], 172, 1811, 0, 180);
        Serial.printf("degrees: %03d, zoom: ", degrees);
        //steering.write(degrees);
        int zoom = map(data.ch[0], 172, 1811, 0, 180);
        //Serial.println(zoom);
        drive.write(zoom);
    }

    // increment used when several values must be sent within the same physical ID - only one per cycle
    static unsigned int i[29] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    while (FrskySP.available ()) {

        uint8_t data = FrskySP.read();
        if (data != 0x7E) {
            if(data != 0x00) {
                Serial.printf("got garbage: %02X\r\n", data);
            }
        } else {
            while (!FrskySP.available ());  // wait for the next byte
            uint8_t sensorRequest = FrskySP.read();
            //Serial.printf("sbus request for %02X\r\n", sensorRequest);
            switch (sensorRequest) {

                case 0x00:  // Physical ID 1 - Vario2 (altimeter high precision)
                    FrskySP.sendData (FRSKY_SP_ALT, 222.2 * 100);                       // 222.2m - warning: OpenTX will automatically apply an auto offset at detection (can be disabled)
                    i[1]++;
                    break;

                case 0xA1:  // Physical ID 2 - FLVSS Lipo sensor (can be sent with one or two cell voltages)
                    // 1st FLVSS
                    if (i[2] % 6 == 0)  FrskySP.sendData (FRSKY_SP_CELLS, FrskySP.lipoCell (0, 3.50, 3.51));
                    if (i[2] % 6 == 1)  FrskySP.sendData (FRSKY_SP_CELLS, FrskySP.lipoCell (2, 3.52, 3.53));
                    if (i[2] % 6 == 2)  FrskySP.sendData (FRSKY_SP_CELLS, FrskySP.lipoCell (4, 3.54, 3.55));

                    // 2nd FLVSS by using logical ID + 1 - could use another physical ID as well
                    if (i[2] % 6 == 3)  FrskySP.sendData (FRSKY_SP_CELLS+1, FrskySP.lipoCell (0, 3.56, 3.57));
                    if (i[2] % 6 == 4)  FrskySP.sendData (FRSKY_SP_CELLS+1, FrskySP.lipoCell (2, 3.58, 3.59));
                    if (i[2] % 6 == 5)  FrskySP.sendData (FRSKY_SP_CELLS+1, FrskySP.lipoCell (4, 3.60, 3.61));

                    i[2]++;
                    break;

                case 0x22:  // Physical ID 3 - FAS-40S current sensor
                    if (i[3] % 2 == 0) FrskySP.sendData (FRSKY_SP_CURR, 11.5 * 10);     // 11.5A
                    if (i[3] % 2 == 1) FrskySP.sendData (FRSKY_SP_VFAS, 22.22 * 100);   // 22.22V
                    i[3]++;
                    break;

                case 0x83:  // Physical ID 4 - GPS / altimeter (normal precision)
                    if (i[4] % 3 == 0) FrskySP.sendData (FRSKY_SP_GPS_ALT, 234.5 * 100);                   // 234.5m
                    if (i[4] % 3 == 1) FrskySP.sendData (FRSKY_SP_GPS_SPEED, (float) 100 / 1.852 * 1000);  // 53.9 kts, 99.9 km/h
                    if (i[4] % 3 == 2) FrskySP.sendData (FRSKY_SP_ALT, 111.1 * 100);                       // 111.1m - warning: OpenTX will automatically apply an auto offset at detection (can be disabled)
                    i[4]++;
                    break;

                case 0xE4:  // Physical ID 5 - RPM
                    if(digitalRead(TEST_SW)) {
                        FrskySP.sendData (FRSKY_SP_RPM, 11111);  // 11111 rpm
                    } else {
                        FrskySP.sendData (FRSKY_SP_RPM, 22222);  // 11111 rpm
                    }
                    i[5]++;
                    break;

                case 0x45:  // Physical ID 6 - SP2UART(Host)
                    if (i[6] % 3 == 0) FrskySP.sendData (FRSKY_SP_A2, 1.1 * 10);      // 0.6 V - set ratio to « - » to get the correct value (one decimal precision only)
                    if (i[6] % 3 == 1) FrskySP.sendData (FRSKY_SP_A3, 11.11 * 100);   // 11.11 V
                    if (i[6] % 3 == 2) FrskySP.sendData (FRSKY_SP_A4, 111.11 * 100);  // 111.11 V
                    i[6]++;
                    break;

                case 0xC6:  // Physical ID 7 - SPUART(Remote)
                    i[7]++;
                    break;

                case 0x67:  // Physical ID 8 -
                    i[8]++;
                    break;

                case 0x48:  // Physical ID 9 -
                    i[9]++;
                    break;

                case 0xE9:  // Physical ID 10 -
                    i[10]++;
                    break;

                case 0x6A:  // Physical ID 11 -
                    i[11]++;
                    break;

                case 0xCB:  // Physical ID 12 -
                    i[12]++;
                    break;

                case 0xAC:  // Physical ID 13 -
                    i[13]++;
                    break;

                case 0x0D:  // Physical ID 14 -
                    i[14]++;
                    break;

                case 0x8E:  // Physical ID 15 -
                    i[15]++;
                    break;

                case 0x2F:  // Physical ID 16 -
                    i[16]++;
                    break;

                case 0xD0:  // Physical ID 17 -
                    i[17]++;
                    break;

                case 0x71:  // Physical ID 18 -
                    i[18]++;
                    break;

                case 0xF2:  // Physical ID 19 -
                    i[19]++;
                    break;

                case 0x53:  // Physical ID 20 -
                    i[20]++;
                    break;

                case 0x34:  // Physical ID 21 -
                    i[21]++;
                    break;

                case 0x95:  // Physical ID 22 -
                    i[22]++;
                    break;

                case 0x16:  // Physical ID 23 -
                    i[23]++;
                    break;

                case 0xB7:  // Physical ID 24 -
                    if (i[24] % 3 == 0) FrskySP.sendData (FRSKY_SP_ACCX,  1.11 * 100);  // 1.11g
                    if (i[24] % 3 == 1) FrskySP.sendData (FRSKY_SP_ACCY, -2.22 * 100);  // -2.22g
                    if (i[24] % 3 == 2) FrskySP.sendData (FRSKY_SP_ACCZ,  3.33 * 100);  // 3.33g
                    i[24]++;
                    break;

                case 0x98:  // Physical ID 25 -
                    i[25]++;
                    break;

                case 0x39:  // Physical ID 26 - Power Box
                    i[26]++;
                    break;

                case 0xBA:  // Physical ID 27 -
                    if (i[27] % 2 == 0) FrskySP.sendData (FRSKY_SP_T1, 28);             // 28 °C
                    if (i[27] % 2 == 1) FrskySP.sendData (FRSKY_SP_T2, 18);             // 18 °C
                    i[27]++;
                    break;

                case 0x1B:  // Physical ID 28 -
                    if (i[28] % 2 == 0) FrskySP.sendData (FRSKY_SP_FUEL, 41);           // 41%
                    if (i[28] % 2 == 1) FrskySP.sendData (FRSKY_SP_FUEL_QTY, 14 * 100); // 14 ml
                    i[28]++;
                    break;
            }
        }
    }
}
