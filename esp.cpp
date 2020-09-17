/*
 * Copyright (C) 2020 Daniel Igaz
 *
 * This file is part of the WiFi ESP click project.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "esp.hpp"
#include "string.h"
#include "stdlib.h"
#include "chprintf.h"

#if defined(DEBUG_OUTPUT_ENABLED) || defined(__DOXYGEN__)
#include "debug.h"
#else
#define DEBUG_PRINT(...) \
	while(0)
#endif

// static singleton instance
ESP ESP::m_instance;

// thread context
THD_WORKING_AREA(ESP::m_thread_ctx, 512);

const ESP::RespMsg ESP::m_respMsg[] = {
	{ "OK", MSG_OK },
	{ "ERROR", MSG_ERROR },
	{ "FAIL", MSG_FAIL },
	{ "SEND OK", MSG_OK },
	{ "SEND FAIL", MSG_FAIL },
	{ NULL, MSG_LINE }
};

ESP::ESP()
: m_serial(NULL)
, m_thread(NULL)
{
	chEvtObjectInit(&m_event);
	chMtxObjectInit(&m_mutex);
	for (uint8_t linkId = 0; linkId < NUM_CHANNELS; linkId++) {
		Channels& chn = m_channels[linkId];
		chn.linkId = linkId;
		iqObjectInit(&chn.iqueue, chn.ibuffer, CHANNEL_BUFFERS_SIZE, NULL, NULL);
	}
}

void ESP::init(SerialDriver& serial, ioline_t rstPin)
{
	m_serial = &serial;
	m_rstPin = rstPin;
}

msg_t ESP::start()
{
	DEBUG_PRINT("");

	msg_t ret;
	static constexpr SerialConfig serialCfg = {
			115200, 0, USART_CR2_STOP1_BITS, 0
	};

	if (m_thread != NULL)
		return MSG_OK;

	m_pbuffer = m_buffer;
	m_buffer[0] = 0;
	m_atParserState = STATE_IDLE;
	sdStart(m_serial, &serialCfg);

	// reset the device
	palClearLine(m_rstPin);
	chThdSleepMilliseconds(10);
	palSetLine(m_rstPin);

	// wait until device ready
	while ((ret = readline(0, nullptr, READY_TIMEOUT)) == MSG_LINE && strcmp(m_buffer, "ready") != 0);

	// disable echoing
	if ((ret = enableEcho(true)) != MSG_OK)
		return ret;

	// set multiplexing mode
	if ((ret = ipEnableMuxMode(true)) != MSG_OK)
		return ret;

	// set SSL buffer size
	if ((ret = ipSetSSLBufferSize()) != MSG_OK)
		return ret;

	// create data and events handling thread
	m_thread = chThdCreateStatic(m_thread_ctx, sizeof(m_thread_ctx), NORMALPRIO + 1, run, (void *) &m_instance);

	return MSG_OK;
}

msg_t ESP::stop()
{
	DEBUG_PRINT("");

	if (m_thread == NULL)
		return MSG_OK;

	chThdTerminate(m_thread);
	chEvtSignal(m_thread, EVENT_MASK(1));
	chThdWait(m_thread);
	m_thread = NULL;

	sdStop(m_serial);

	return MSG_OK;
}

void ESP::run(void *args)
{
	ESP *instance = (ESP *)args;
	instance->run();
}

void ESP::run()
{
	chRegSetThreadName("ESP::run");

	event_listener_t evtl;

	chEvtRegisterMaskWithFlags(&m_serial->event, &evtl, EVENT_MASK(0), CHN_INPUT_AVAILABLE);

	while (!chThdShouldTerminateX()) {
		eventmask_t mask = chEvtWaitAny(ALL_EVENTS);
		if (mask & EVENT_MASK(0)) {
			eventflags_t flags = chEvtGetAndClearFlags(&evtl);
			if (flags & CHN_INPUT_AVAILABLE) {
				if (chMtxTryLock(&m_mutex) == true) {
					readline(0, nullptr, TIME_IMMEDIATE);
					chMtxUnlock(&m_mutex);
				}
			}
		}
	}

	chEvtUnregister(&m_serial->event, &evtl);
}

msg_t ESP::readline(msg_t mask, std::function<void()> cb, sysinterval_t timeout)
{
	while (true) {
		msg_t c = chnGetTimeout(m_serial, timeout);
		if (c == MSG_TIMEOUT || c == MSG_RESET) {
			// reset the state machine
			m_atParserState = STATE_IDLE;
			m_pbuffer = m_buffer;
			return c;
		}

		// from now on use character timeout
		timeout = CHAR_TIMEOUT;

		switch (m_atParserState) {
		case STATE_IDLE:
			if (c == '\r')
				break;

			if (c == '\n') {
				if (m_pbuffer == m_buffer) {
					m_atParserState = STATE_WAITING_FOR_RESPONSE;
					break;
				}

				// checking for event messages and callback
				if (strcmp(m_buffer, "WIFI CONNECTED") == 0) {
					//DEBUG_PRINT("wifi connected");
					chEvtBroadcastFlags(&m_event, WIFI_CONNECTED);
				}
				else if (strcmp(m_buffer, "WIFI DISCONNECT") == 0) {
					//DEBUG_PRINT("wifi disconnected");
					chEvtBroadcastFlags(&m_event, WIFI_DISCONNECTED);
				}
				else if (strcmp(m_buffer, "WIFI GOT IP") == 0) {
					//DEBUG_PRINT("wifi ip address assigned");
					chEvtBroadcastFlags(&m_event, WIFI_GOT_IP_ADDRESS);
				}
				else if (strstr(m_buffer, ",CONNECT") != NULL) {
					//DEBUG_PRINT("client connected");
					chEvtBroadcastFlags(&m_event, CHN_CONNECTED);
				}
				else if (strstr(m_buffer, ",CLOSED") != NULL) {
					//DEBUG_PRINT("client disconnected");
					chEvtBroadcastFlags(&m_event, CHN_DISCONNECTED);
				}
				else if (cb != NULL)
					cb();

				m_atParserState = STATE_IDLE;
				m_pbuffer = m_buffer;
				return MSG_LINE;
			}
			else if (m_pbuffer < m_buffer + BUFFER_SIZE) {
				*m_pbuffer++ = c;
				*m_pbuffer = 0;
			}
			else {
				// reset the state machine, eventually it will timeout
				m_atParserState = STATE_IDLE;
				m_pbuffer = m_buffer;
			}
			break;
		case STATE_WAITING_FOR_RESPONSE:
			if (c == '\r')
				break;

			if (c == '\n') {
				// check for responses
				const RespMsg* respMsg = m_respMsg;
				while (mask && respMsg->msg != MSG_LINE) {
					if (mask & (1 << respMsg->msg) && strcmp(m_buffer, respMsg->name) == 0)
						break;
					respMsg++;
				}

				m_atParserState = STATE_IDLE;
				m_pbuffer = m_buffer;
				return respMsg->msg;
			}
			else if (m_pbuffer < m_buffer + BUFFER_SIZE) {
				*m_pbuffer++ = c;
				*m_pbuffer = 0;

				if (strncmp(m_buffer, "+IPD", 4) == 0 && c == ':') {
					char* saveptr;
					strtok_r(m_buffer, ",", &saveptr);
					m_linkId = atoi(strtok_r(NULL, ",", &saveptr));
					m_length = atoi(strtok_r(NULL, ":", &saveptr));

					if (m_length > 0)
						m_atParserState = STATE_DATA_MODE;
					else
						m_atParserState = STATE_IDLE;
					m_pbuffer = m_buffer;
				}
			}
			else {
				// reset the state machine, eventually it will timeout
				m_atParserState = STATE_IDLE;
				m_pbuffer = m_buffer;
			}
			break;
		case STATE_DATA_MODE:
			if (m_linkId < NUM_CHANNELS) {
				osalSysLock();
				iqPutI(&m_channels[m_linkId].iqueue, c);
				osalSysUnlock();
			}

			if (--m_length == 0) {
				m_atParserState = STATE_IDLE;
				return MSG_TIMEOUT;
			}
			break;
		}
	}
}

msg_t ESP::command(msg_t mask, std::function<void()> cb, sysinterval_t timeout, const char *fmt, ...)
{
	osalDbgAssert(mask != 0, "no mask");

	va_list ap;
	msg_t ret;

	chMtxLock(&m_mutex);

	va_start(ap, fmt);
	chvprintf((BaseSequentialStream *)m_serial, fmt, ap);
	va_end(ap);

	do {
		ret = readline(mask, cb, timeout);
	} while (ret == MSG_LINE);

	chMtxUnlock(&m_mutex);

	DEBUG_PRINT("ret = %d", ret);
	return ret;
}

msg_t ESP::ipGetStatus(Status& status)
{
	DEBUG_PRINT("");

	status = Status::STATUS_UNKNOWN;

	auto getStatusCb = [this, &status]() {
		char *saveptr;

		if (strncmp(m_buffer, "STATUS:", strlen("STATUS:")) == 0) {
			// status
			status = (Status)atoi(strtok_r(m_buffer + strlen("STATUS:"), "", &saveptr));
		}

		else if (strncmp(m_buffer, "+CIPSTATUS:", strlen("+CIPSTATUS:")) == 0) {
			// link id
			uint8_t linkId = atoi(strtok_r(m_buffer + strlen("+CIPSTATUS:"), ",", &saveptr));

			if (linkId < NUM_CHANNELS) {
				Channels& chn = m_channels[linkId];
				char *pstart, *pend;

				// TCP/UDP/SSL
				chn.type = CHN_TYPE_UNKNOWN;
				pstart = strtok_r(NULL, ",", &saveptr);
				if ((pstart != NULL) && ((pstart = strchr(pstart, '"')) != NULL) && ((pend = strrchr(pstart, '"')) != NULL) && (pstart != pend)) {
					pstart++;
					*pend = 0;
					if (strcmp("\"TCP\"", pstart) == 0)
						chn.type = CHN_TYPE_TCP;
					else if (strcmp("UDP", pstart) == 0)
						chn.type = CHN_TYPE_UDP;
					else if (strcmp("SSL", pstart) == 0)
						chn.type = CHN_TYPE_SSL;
				}

				// remote IP
				memset(chn.remoteAddr, 0, IP_ADDRESS_SIZE);
				pstart = strtok_r(NULL, ",", &saveptr);
				if ((pstart != NULL) && ((pstart = strchr(pstart, '"')) != NULL) && ((pend = strrchr(pstart, '"')) != NULL) && (pstart != pend)) {
					pstart++;
					*pend = 0;
					strncpy(chn.remoteAddr, pstart, IP_ADDRESS_SIZE - 1);
				}

				// remote port
				chn.remotePort = 0;
				pstart = strtok_r(NULL, ",", &saveptr);
				if (pstart != NULL)
					chn.remotePort = atoi(pstart);

				// local port
				chn.localPort = 0;
				pstart = strtok_r(NULL, ",", &saveptr);
				if (pstart != NULL)
					chn.localPort = atoi(pstart);
			}
		}
	};

	return command((1 << MSG_OK) | (1 << MSG_ERROR),
			getStatusCb, RESP_TIMEOUT,
			"AT+CIPSTATUS\r\n");
}

msg_t ESP::reset()
{
	DEBUG_PRINT("");

	// reset module
	msg_t ret = command((1 << MSG_OK) | (1 << MSG_ERROR),
			nullptr, RESP_TIMEOUT,
			"AT+RST\r\n");
	if (ret != MSG_OK)
		return ret;

	// wait until device ready
	while ((ret = readline(0, nullptr, READY_TIMEOUT)) == MSG_LINE && strcmp(m_buffer, "ready") != 0);

	return ret;
}

msg_t ESP::enableEcho(bool enable)
{
	DEBUG_PRINT("enable = %c", enable ? 'T' : 'F');

	return command((1 << MSG_OK) | (1 << MSG_ERROR),
			nullptr, RESP_TIMEOUT,
			"ATE%d\r\n", enable ? 1 : 0);
}

msg_t ESP::enterDeepSleep(uint32_t timeout)
{
	DEBUG_PRINT("timeout = %lu", timeout);

	return command((1 << MSG_OK) | (1 << MSG_ERROR),
			nullptr, RESP_TIMEOUT,
			"AT+GSLP=%lu\r\n", timeout);
}

msg_t ESP::ipEnableMuxMode(bool enable)
{
	DEBUG_PRINT("enable = %c", enable ? 'T' : 'F');

	return command((1 << MSG_OK) | (1 << MSG_ERROR),
			nullptr, RESP_TIMEOUT,
			"AT+CIPMUX=%d\r\n", enable ? 1 : 0);
}

msg_t ESP::ipSetSSLBufferSize(uint16_t size)
{
	DEBUG_PRINT("size = %u", size);

	return command((1 << MSG_OK) | (1 << MSG_ERROR),
			nullptr, RESP_TIMEOUT,
			"AT+CIPSSLSIZE=%u\r\n", size);
}

msg_t ESP::wifiSetMode(WifiMode mode, bool defaultCfg)
{
	DEBUG_PRINT("mode = %d, defaultCfg = %c", mode, defaultCfg ? 'T' : 'F');

	return command((1 << MSG_OK) | (1 << MSG_ERROR),
			nullptr, RESP_TIMEOUT,
			"AT+CWMODE_%s=%d\r\n", defaultCfg ? "DEF" : "CUR", mode);
}

msg_t ESP::wifiConnect(const char* ssid, const char* pwd, bool defaultCfg)
{
	osalDbgCheck((ssid != NULL) && (pwd != NULL));

	DEBUG_PRINT("ssid = \"%s\", pwd = \"%s\", defaultCfg = %c", ssid, pwd, defaultCfg ? 'T' : 'F');

	msg_t errorCode = MSG_FAIL;
	msg_t ret;

	auto stationConnectCb = [this, &errorCode]() {
		char *saveptr;

		if (strncmp(this->m_buffer, "+CWJAP:", strlen("+CWJAP:")) == 0) {
			// error code
			errorCode = atoi(strtok_r(this->m_buffer + strlen("+CWJAP:"), ",", &saveptr));
			switch(errorCode) {
			case 1:
				DEBUG_PRINT("connection timeout");
				errorCode = MSG_CONN_TIMEOUT;
				break;
			case 2:
				DEBUG_PRINT("wrong password");
				errorCode = MSG_CONN_INVALID_PWD;
				break;
			case 3:
				DEBUG_PRINT("cannot find target access point");
				errorCode = MSG_CONN_INVALID_AP;
				break;
			case 4:
				DEBUG_PRINT("connection failed");
				errorCode = MSG_CONN_FAILED;
				break;
			default:
				break;
			}
		}
	};

	ret = command((1 << MSG_OK) | (1 << MSG_ERROR) | (1 << MSG_FAIL),
			stationConnectCb, CONNECT_TIMEOUT,
			"AT+CWJAP_%s=\"%s\",\"%s\"\r\n", defaultCfg ? "DEF" : "CUR", ssid, pwd);
	if (ret == MSG_FAIL)
		ret = errorCode;

	return ret;
}

msg_t ESP::wifiDisconnect()
{
	DEBUG_PRINT("");

	return command((1 << MSG_OK) | (1 << MSG_ERROR),
			nullptr, RESP_TIMEOUT,
			"AT+CWQAP\r\n");
}

msg_t ESP::wifiAutoConnect(bool enable)
{
	DEBUG_PRINT("enable = %c", enable ? 'T' : 'F');

	return command((1 << MSG_OK) | (1 << MSG_ERROR),
			nullptr, RESP_TIMEOUT,
			"AT+CWAUTOCONN=%u\r\n", enable ? 1 : 0);
}

msg_t ESP::wifiSetAP(const char *ssid, const char *pwd, uint8_t chn, WifiEnc enc, uint8_t maxConn, bool hidden, bool defaultCfg)
{
	osalDbgCheck((ssid != NULL) && (pwd != NULL));

	DEBUG_PRINT("ssid = \"%s\", pwd = \"%s\", chn = %u, enc = %u, maxConn = %u, hidden = %c, defaultCfg = %c",
			ssid, pwd, chn, enc, maxConn, hidden ? 'T' : 'F', defaultCfg ? 'T' : 'F');

	return command((1 << MSG_OK) | (1 << MSG_ERROR),
				nullptr, RESP_TIMEOUT,
				"AT+CWSAP_%s=\"%s\",\"%s\",%u,%u,%u,%u\r\n",
				defaultCfg ? "DEF" : "CUR", ssid, pwd, chn, enc, maxConn, hidden ? 1 : 0);
}

msg_t ESP::wifiEnableDHCP(WifiMode mode, bool enable, bool defaultCfg)
{
	DEBUG_PRINT("mode = %u, enable = %c, defaultCfg = %c", enable ? 'T' : 'F', defaultCfg ? 'T' : 'F');

	return command((1 << MSG_OK) | (1 << MSG_ERROR),
			nullptr, RESP_TIMEOUT,
			"AT+CWDHCP_%s=%u,%u\r\n", defaultCfg ? "DEF" : "CUR", mode, enable ? 1 : 0);
}

msg_t ESP::wifiSetDHCPIpRange(uint16_t leaseTime, const char *startIp, const char *endIp, bool defaultCfg)
{
	osalDbgCheck((startIp != NULL) && (endIp != NULL));

	DEBUG_PRINT("leaseTime = %u, startIp = \"%s\", endIp = \"%s\", defaultCfg = %c",
			leaseTime, startIp, endIp, defaultCfg ? 'T' : 'F');

	return command((1 << MSG_OK) | (1 << MSG_ERROR),
			nullptr, RESP_TIMEOUT,
			"AT+CWDHCPS_%s=1,%d,\"%s\",\"%s\"\r\n", defaultCfg ? "DEF" : "CUR", leaseTime, startIp, endIp);
}

msg_t ESP::wifiSetDHCPIpRange(bool defaultCfg)
{
	DEBUG_PRINT("defaultCfg = %c", defaultCfg ? 'T' : 'F');

	return command((1 << MSG_OK) | (1 << MSG_ERROR),
			nullptr, RESP_TIMEOUT,
			"AT+CWDHCPS_%s=0\r\n", defaultCfg ? "DEF" : "CUR");
}

msg_t ESP::ipConnect(uint8_t linkId, const char *host, uint16_t remotePort, uint16_t keepAlive, bool secure)
{
	osalDbgCheck(host != NULL);

	DEBUG_PRINT("linkId = %u, host = \"%s\", remotePort = %u, keepAlive = %u, secure = %c",
			linkId, host, remotePort, keepAlive, secure ? 'T' : 'F');

	return command((1 << MSG_OK) | (1 << MSG_ERROR),

			nullptr, RESP_TIMEOUT,
			"AT+CIPSTART=%u,\"%s\",\"%s\",%u,%u\r\n",
			linkId, secure ? "SSL" : "TCP", host, remotePort, keepAlive);
}

msg_t ESP::ipConnect(uint8_t linkId, const char *host, uint16_t remotePort, uint16_t localPort, UdpMode mode)
{
	osalDbgCheck(host != NULL);

	DEBUG_PRINT("linkId = %u, host = \"%s\", remotePort = %u, localPort = %u, mode = %u",
			linkId, host, remotePort, localPort, mode);

	return command((1 << MSG_OK) | (1 << MSG_ERROR),
			nullptr, RESP_TIMEOUT,
			"AT+CIPSTART=%d,\"UDP\",\"%s\",%u,%u,%u\r\n",
			linkId, host, remotePort, localPort, mode);
}

msg_t ESP::ipDisconnect(uint8_t linkId)
{
	DEBUG_PRINT("linkId = %u", linkId);

	return command((1 << MSG_OK) | (1 << MSG_ERROR),
			nullptr, RESP_TIMEOUT,
			"AT+CIPCLOSE=%u\r\n", linkId);
}

bool ESP::ipSendDataHeader(uint8_t linkId, size_t n)
{
	msg_t ret;

	// send header
	chprintf((BaseSequentialStream *)m_serial, "AT+CIPSEND=%u,%u\r\n", linkId, n);

	do {
		ret = readline((1 << MSG_OK) | (1 << MSG_ERROR), nullptr, RESP_TIMEOUT);
	} while (ret == MSG_LINE);

	if (ret != MSG_OK)
		return false;

	// wait for prompt
	memset(m_buffer, 0, BUFFER_SIZE);
	ret = chnReadTimeout(m_serial, (uint8_t*)m_buffer, strlen("> "), RESP_TIMEOUT);
	if (strcmp(m_buffer, "> ") != 0)
		return false;

	return true;
}

size_t ESP::ipSendData(uint8_t linkId, const uint8_t *buf, size_t n)
{
//	DEBUG_PRINT("linkId = %u", linkId);

	msg_t ret;
	size_t size = 0;

	chMtxLock(&m_mutex);

	while (n > 0) {
		// send header
		size_t bytesToSend = (n > CHN_MAX_BUFFER_SIZE) ? CHN_MAX_BUFFER_SIZE : n;
		if (!ipSendDataHeader(linkId, bytesToSend))
			break;

		// send data
		if (sdWrite(m_serial, buf, n) != n)
			break;

		do {
			ret = readline((1 << MSG_OK) | (1 << MSG_ERROR), nullptr, RESP_TIMEOUT);
		} while (ret == MSG_LINE);

		if (ret != MSG_OK)
			break;

		size += bytesToSend;
		n -= bytesToSend;
	}

	chMtxUnlock(&m_mutex);

	return size;
}

size_t ESP::ipReceiveData(uint8_t linkId, uint8_t *buf, size_t n, sysinterval_t timeout)
{
//	DEBUG_PRINT("linkId = %u, timeout = %lu", linkId, timeout);

	return iqReadTimeout(&m_channels[linkId].iqueue, buf, n, timeout);
}

msg_t ESP::ipReceiveData(uint8_t linkId, sysinterval_t timeout)
{
	return iqGetTimeout(&m_channels[linkId].iqueue, timeout);
}

msg_t ESP::ipCreateServer(uint16_t localPort)
{
	DEBUG_PRINT("localPort = %u", localPort);

	return command((1 << MSG_OK) | (1 << MSG_ERROR),
			nullptr, RESP_TIMEOUT,
			"AT+CIPSERVER=1,%u\r\n", localPort);
}

msg_t ESP::ipDeleteServer()
{
	DEBUG_PRINT("");

	return command((1 << MSG_OK) | (1 << MSG_ERROR),
			nullptr, RESP_TIMEOUT,
			"AT+CIPSERVER=0\r\n");
}

msg_t ESP::ipGetAddres(std::string& apIPAddr, std::string& apMACAddr, std::string& staIPAddr, std::string& staMACAddr)
{
	DEBUG_PRINT("");

	auto getAddressCb = [this, &apIPAddr, &apMACAddr, &staIPAddr, &staMACAddr]() {
		struct KeyValuePair {
			const char *key;
			std::string& value;
		} pair[] = {
				{ "+CIFSR:APIP", apIPAddr },
				{ "+CIFSR:APMAC", apMACAddr },
				{ "+CIFSR:STAIP", apIPAddr },
				{ "+CIFSR:STAMAC", apMACAddr },
		};

		for (size_t i = 0; i < sizeof(pair) / sizeof(KeyValuePair); i++) {
			if (strncmp(this->m_buffer, pair[i].key, strlen(pair[i].key)) == 0) {
				char *saveptr = this->m_buffer + strlen(pair[i].key);
				const char *pstart = strtok_r(saveptr, ",", &saveptr);
				const char *pend;
				if ((pstart != NULL) && ((pstart = strchr(pstart, '"')) != NULL) && ((pend = strrchr(pstart, '"')) != NULL) && (pstart != pend))
					pair[i].value = std::string(++pstart, pend);
				break;
			}
		}
	};

	return command((1 << MSG_OK) | (1 << MSG_ERROR),
			getAddressCb, RESP_TIMEOUT,
			"AT+CIFSR\r\n");
}