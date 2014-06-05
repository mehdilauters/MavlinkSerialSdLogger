
#include <AP_Common.h>
#include <AP_Math.h>


#include "../GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h"
#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"

// true when we have received at least 1 MAVLink packet
static bool mavlink_active;
static uint8_t crlf_count = 0;

static int packet_drops = 0;
static int parse_error = 0;

static uint8_t      apm_mav_system; 
static uint8_t      apm_mav_component;

void request_mavlink_rates()
{
    const int  maxStreams = 7;
    const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_RAW_SENSORS,
        MAV_DATA_STREAM_EXTENDED_STATUS,
        MAV_DATA_STREAM_RC_CHANNELS,
        MAV_DATA_STREAM_POSITION,
        MAV_DATA_STREAM_EXTRA1, 
        MAV_DATA_STREAM_EXTRA2,
      MAV_DATA_STREAM_EXTRA3};
    const uint16_t MAVRates[maxStreams] = {0x02, 0x02, 0x05, 0x02, 0x05, 0x02, 0x02};
    for (int i=0; i < maxStreams; i++) {
        mavlink_msg_request_data_stream_send(MAVLINK_COMM_0,
            apm_mav_system, apm_mav_component,
            MAVStreams[i], MAVRates[i], 1);
    }
}

void read_mavlink(){
    mavlink_message_t msg; 
    mavlink_status_t status;
    //grabing data 
    while(Serial1.available() > 0) { 
        uint8_t c = Serial1.read();
        // log to file raw data
         if (dataFile) {
                uint8_t res =  dataFile.write(c);
                  if (res != 1)
                  {
                      digitalWrite(LED_PIN, HIGH);
                      delay(FILE_ERROR_DELAY);
                      digitalWrite(LED_PIN, LOW);
                      delay(FILE_ERROR_DELAY);
                      Serial.println("I/O error");
                  }
        }
        

        //trying to grab msg  
        if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
          if (dataFile) {
            dataFile.flush();
          }
            mavlink_active = 1;
            //handle msg
            switch(msg.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT:
                {
                    apm_mav_system    = msg.sysid;
                    apm_mav_component = msg.compid;
                 if(waitingMAVBeats == 1){
                  enable_mav_request = 1;
                 }
                }
                break;
                case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
                {
                  int16_t chan6Value = mavlink_msg_rc_channels_raw_get_chan6_raw(&msg);
                  if(chan6Value < 1600) // off
                  {
                     digitalWrite(BUZZER_PIN , LOW);
                  }
                  else
                  {
                     digitalWrite(BUZZER_PIN , HIGH);
                  }
                }
             break;
            default:
                //Do nothing
                break;
            }
        }
        delayMicroseconds(138);
        //next one
    }
    // Update global packet drops counter
    packet_drops += status.packet_rx_drop_count;
    parse_error += status.parse_error;

}
