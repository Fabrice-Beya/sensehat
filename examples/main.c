#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <MQTTClient.h>

#define ADDRESS     "tcp://192.168.3.81:1883"
#define CLIENTID    "RaspberryPiClient"
#define QOS         1
#define TIMEOUT     10000L

MQTTClient client;
MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;

IMU_EN_SENSOR_TYPE enMotionSensorType;
IMU_ST_ANGLES_DATA stAngles;
IMU_ST_SENSOR_DATA stGyroRawData;
IMU_ST_SENSOR_DATA stAccelRawData;
IMU_ST_SENSOR_DATA stMagnRawData;

void Handler(int signo) {
    printf("\r\nHandler:exit\r\n");
    MQTTClient_disconnect(client, 10000);
    MQTTClient_destroy(&client);
    DEV_ModuleExit();
    exit(0);
}

void publish_sensor_data(const char* topic, const char* payload) {
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    MQTTClient_deliveryToken token;
    pubmsg.payload = (void*)payload;
    pubmsg.payloadlen = (int)strlen(payload);
    pubmsg.qos = QOS;
    pubmsg.retained = 0;
    MQTTClient_publishMessage(client, topic, &pubmsg, &token);
    MQTTClient_waitForCompletion(client, token, TIMEOUT);
}

char* get_current_time() {
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    static char buf[80];
    strftime(buf, sizeof(buf)-1, "%Y-%m-%d %H:%M:%S", t);
    return buf;
}

int main(void) {
    signal(SIGINT, Handler);
    DEV_ModuleInit();
    
    MQTTClient_create(&client, ADDRESS, CLIENTID, MQTTCLIENT_PERSISTENCE_NONE, NULL);
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;

    int rc;
    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS) {
        switch (rc) {
            case MQTTCLIENT_FAILURE:
                printf("Failed to connect to MQTT broker: General failure\n");
                break;
            case MQTTCLIENT_DISCONNECTED:
                printf("Failed to connect to MQTT broker: Disconnected\n");
                break;
            case MQTTCLIENT_MAX_MESSAGES_INFLIGHT:
                printf("Failed to connect to MQTT broker: Max messages in flight\n");
                break;
            case MQTTCLIENT_BAD_UTF8_STRING:
                printf("Failed to connect to MQTT broker: Bad UTF-8 string\n");
                break;
            case MQTTCLIENT_NULL_PARAMETER:
                printf("Failed to connect to MQTT broker: Null parameter\n");
                break;
            case MQTTCLIENT_TOPICNAME_TRUNCATED:
                printf("Failed to connect to MQTT broker: Topic name truncated\n");
                break;
            case MQTTCLIENT_BAD_STRUCTURE:
                printf("Failed to connect to MQTT broker: Bad structure\n");
                break;
            case MQTTCLIENT_BAD_QOS:
                printf("Failed to connect to MQTT broker: Bad QoS\n");
                break;
            default:
                printf("Failed to connect to MQTT broker: Unknown error code %d\n", rc);
                break;
        }
        exit(EXIT_FAILURE);
    }
    
    uint8_t light;
    BME280_Init();
    TSL2591_Init();
    LTR390_init();
    SGP40_init();
    imuInit(&enMotionSensorType);
    
    char payload[256];
    
    while (1) {
        BME280_value();
        light = TSL2591_Read_Lux();
        UVS_value();
        SGP40_value();
        imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);

        snprintf(payload, sizeof(payload), "{value: %.2f, timeStamp: \"%s\"}", pres_raw[1], get_current_time());
        publish_sensor_data("senshub/temperature", payload);

        snprintf(payload, sizeof(payload), "{value: %d, timeStamp: \"%s\"}", light, get_current_time());
        publish_sensor_data("senshub/light", payload);

        snprintf(payload, sizeof(payload), "{value: %d, timeStamp: \"%s\"}", uv, get_current_time());
        publish_sensor_data("senshub/uv", payload);

        snprintf(payload, sizeof(payload), "{value: %d, timeStamp: \"%s\"}", gas, get_current_time());
        publish_sensor_data("senshub/gas", payload);

        snprintf(payload, sizeof(payload), "{value: {roll: %.2f, pitch: %.2f, yaw: %.2f}, timeStamp: \"%s\"}",
                 stAngles.fRoll, stAngles.fPitch, stAngles.fYaw, get_current_time());
        publish_sensor_data("senshub/angles", payload);

        snprintf(payload, sizeof(payload), "{value: {x: %d, y: %d, z: %d}, timeStamp: \"%s\"}",
                 stAccelRawData.s16X, stAccelRawData.s16Y, stAccelRawData.s16Z, get_current_time());
        publish_sensor_data("senshub/acceleration", payload);

        snprintf(payload, sizeof(payload), "{value: {x: %d, y: %d, z: %d}, timeStamp: \"%s\"}",
                 stGyroRawData.s16X, stGyroRawData.s16Y, stGyroRawData.s16Z, get_current_time());
        publish_sensor_data("senshub/gyroscope", payload);

        snprintf(payload, sizeof(payload), "{value: {x: %d, y: %d, z: %d}, timeStamp: \"%s\"}",
                 stMagnRawData.s16X, stMagnRawData.s16Y, stMagnRawData.s16Z, get_current_time());
        publish_sensor_data("senshub/magnetic", payload);

        DEV_Delay_ms(500);
    }

    MQTTClient_disconnect(client, 10000);
    MQTTClient_destroy(&client);
    DEV_ModuleExit();
    return 0;
}