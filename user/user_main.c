/*
* Copyright (c) 2014-2015, Tuan PM <tuanpm at live dot com>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Redis nor the names of its contributors may be used
* to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#include "ets_sys.h"
#include "driver/uart.h"
#include "osapi.h"
#include "mqtt.h"
#include "wifi.h"
#include "config.h"
#include "debug.h"
#include "gpio.h"
#include "user_interface.h"
#include "mem.h"

MQTT_Client mqttClient;

static volatile os_timer_t gpioStateTimer;

typedef enum {
    FRC1_SOURCE = 0,
    NMI_SOURCE = 1,
} FRC1_TIMER_SOURCE_TYPE;

void ICACHE_FLASH_ATTR wifiConnectCb(uint8_t status){
	if(status == STATION_GOT_IP){
		MQTT_Connect(&mqttClient);
	} else {
		MQTT_Disconnect(&mqttClient);
	}
}

void ICACHE_FLASH_ATTR mqttConnectedCb(uint32_t *args){
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Connected\n");
	MQTT_Subscribe(client, MQTT_TOPIC_GPIO_CONTROL, 0);
}

void ICACHE_FLASH_ATTR mqttDisconnectedCb(uint32_t *args){
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Disconnected\n");
}

void ICACHE_FLASH_ATTR mqttPublishedCb(uint32_t *args){
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Published\n");
}

void ICACHE_FLASH_ATTR mqttParseMessage(const char* topic, uint32_t topic_len, const char *data, uint32_t data_len){
	if(strcmpi(topic, MQTT_TOPIC_GPIO_CONTROL) == 0){
        parseGpioContolMessage(data, data_len);
	}
	else{
		INFO("MQTT: Unknown topic\n");
	}
}

void ICACHE_FLASH_ATTR mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, const char *data, uint32_t data_len){
	char *topicBuf = (char*)os_zalloc(topic_len+1);
	char *dataBuf = (char*)os_zalloc(data_len+1);

	MQTT_Client* client = (MQTT_Client*)args;

	os_memcpy(topicBuf, topic, topic_len);
	topicBuf[topic_len] = 0;

	os_memcpy(dataBuf, data, data_len);
	dataBuf[data_len] = 0;

	INFO("MQTT: Receive topic: %s, data: %s\n", topicBuf, dataBuf);

	mqttParseMessage(topicBuf, topic_len, dataBuf, data_len);

	os_free(topicBuf);
	os_free(dataBuf);
}

void ICACHE_FLASH_ATTR controlGpio(int32_t gpio, int32_t state, int32_t timer){    
    os_timer_disarm(&gpioStateTimer);
     
    if(state > 0){
        gpio_output_set(BIT2, 0, BIT2, 0);
        if(timer > 0){
            os_timer_arm(&gpioStateTimer, timer, 0);
        }
    }else{
        gpio_output_set(0, BIT2, BIT2, 0);        
    }    
}

void ICACHE_FLASH_ATTR parseGpioContolMessage(const char *data, uint32_t data_len){
    uint32_t i;
	int32_t r;
	jsmn_parser p;
	jsmntok_t t[15];

	bool gpioAssigned = true;
	bool stateAssigned = false;
	bool timerAssigned = true;

	int32_t gpio = 2, state = 0, timer = 0;

    jsmn_init(&p);
    r = jsmn_parse(&p, data, data_len, t, sizeof(t)/sizeof(t[0]));

    if (r < 0) {
        INFO("JSON, Parser error: %d\n", r);
        return;
    }

    if (r < 1 || t[0].type != JSMN_OBJECT) {
        INFO("JSON, Object expected\n");
        return;
    }

    if((r != 7) && (r != 6) && (r != 5)){
        INFO("JSON, Missing token(s): %d\n", r);
        return;
    }

    INFO("JSON payload \n");

    for (i = 1; i < r; i++) {
        if (jsonEq(data, &t[i], "gpio")) {
            gpio = strtol(data + t[i+1].start, NULL, 10);
            gpioAssigned = true;
            INFO("\t gpio: %d\n", gpio);
            i++;
        } else if (jsonEq(data, &t[i], "state")) {
            state = strtol(data + t[i+1].start, NULL, 10);
            stateAssigned = true;
            INFO("\t state: %d\n", state);
            i++;
        } else if (jsonEq(data, &t[i], "timer")) {
            timer = strtol(data + t[i+1].start, NULL, 10);				
            timerAssigned = true;
            INFO("\t timer: %d\n", timer);
            i++;
        } else {
            INFO("\tUnexpected key\n");
        }	
    }

    if(gpioAssigned && stateAssigned && timerAssigned){
        INFO("JSON, All parameters assigned for GPIO\n");
        controlGpio(gpio, state, timer);
    }
}

bool ICACHE_FLASH_ATTR jsonEq(const char *json, jsmntok_t *tok, const char *s) {
	return (bool)(tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start && strncmpi(json + tok->start, s, tok->end - tok->start) == 0);
}

void ICACHE_FLASH_ATTR gpioSetup(void){
    gpio_init();
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);
	gpio_output_set(0, BIT2, BIT2, 0);
}

void ICACHE_FLASH_ATTR gpioTimerCb(uint32_t *args){
    os_timer_disarm(&gpioStateTimer);   
    gpio_output_set(0, BIT2, BIT2, 0); 
}

void ICACHE_FLASH_ATTR timerSetup(void){
    os_timer_disarm(&gpioStateTimer);
    os_timer_setfn(&gpioStateTimer, (os_timer_func_t *)gpioTimerCb, NULL);
}

void ICACHE_FLASH_ATTR user_init(void){
	system_timer_reinit();

	uart_init(BIT_RATE_115200, BIT_RATE_115200);
	os_delay_us(1000000);

	INFO("SDK version: %s\n", system_get_sdk_version());
	INFO("System init ...\n");

	CFG_Load();

	gpioSetup();

	MQTT_InitConnection(&mqttClient, sysCfg.mqtt_host, sysCfg.mqtt_port, sysCfg.security);
	MQTT_InitClient(&mqttClient, sysCfg.device_id, sysCfg.mqtt_user, sysCfg.mqtt_pass, sysCfg.mqtt_keepalive, 1);
	MQTT_InitLWT(&mqttClient, "/lwt", "offline", 0, 0);
	MQTT_OnConnected(&mqttClient, mqttConnectedCb);
	MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
	MQTT_OnPublished(&mqttClient, mqttPublishedCb);
	MQTT_OnData(&mqttClient, mqttDataCb);

	WIFI_Connect(sysCfg.sta_ssid, sysCfg.sta_pwd, wifiConnectCb);

	timerSetup();

	INFO("\nSystem started ...\n");
}