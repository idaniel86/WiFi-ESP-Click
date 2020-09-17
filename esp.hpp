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

#ifndef ESP_HPP_
#define ESP_HPP_

#include "hal.h"
#include <functional>

class ESP {
public:
	typedef enum {
		MODE_STATION = 1,
		MODE_AP = 2,
		MODE_STATION_AND_AP = 3,
	} WifiMode;

	typedef enum {
		ENC_OPEN = 0,
		ENC_WPA_PSK = 2,
		ENC_WPA2_PSK = 3,
		ENC_WPA_WPA2_PSK = 4,
	} WifiEnc;

	typedef enum {
		STATUS_UNKNOWN = -1,
		STATUS_WIFI_CONNECTED = 2,
		STATUS_CHN_CONNECTED = 3,
		STATUS_CHN_DISCONNECTED = 4,
		STATUS_WIFI_DISCONNECTED = 5,
	} Status;

	typedef enum {
		UDP_PEER_CANNOT_CHANGE,
		UDP_PEER_CHANGES_ONCE,
		UDP_PEER_CHANGES
	} UdpMode;

	typedef enum {
		CHN_TYPE_TCP = 0,
		CHN_TYPE_UDP = 1,
		CHN_TYPE_SSL = 2,
		CHN_TYPE_UNKNOWN,
	} ChannelType;

	enum : msg_t {
		MSG_LINE = -3,
		MSG_ERROR = 1,
		MSG_FAIL = 2,
		MSG_CONN_TIMEOUT = 3,
		MSG_CONN_INVALID_PWD = 4,
		MSG_CONN_INVALID_AP = 5,
		MSG_CONN_FAILED = 6
	};

	enum : eventflags_t {
		WIFI_CONNECTED = 0x00000004,
		WIFI_DISCONNECTED = 0x00000008,
		WIFI_GOT_IP_ADDRESS = 0x00000010,
	};

private:
	enum : size_t {
		/**
		 * @brief	Line buffer size
		 */
		BUFFER_SIZE = 256,

		/**
		 * @brief	ESP8266 number of channels allocated
		 */
		NUM_CHANNELS = 5,

		/**
		 * @brief   ESP8266 channel buffer size
		 */
		CHANNEL_BUFFERS_SIZE = 16,

		/**
		 * @brief	Maximum size of IP address string
		 */
		IP_ADDRESS_SIZE = 100,

		/**
		 * @brief	Channel buffer size
		 */
		CHN_MAX_BUFFER_SIZE = (size_t)2048,
	};

	enum : sysinterval_t {
		/**
		 * @brief	Character timeout
		 */
		CHAR_TIMEOUT = TIME_MS2I(200),

		/**
		 * @brief	Default response timeout
		 */
		RESP_TIMEOUT = TIME_MS2I(1000),

		/**
		 * @brief	Ready timeout
		 */
		READY_TIMEOUT = TIME_MS2I(5000),

		/**
		 * @brief	Connect timeout
		 */
		CONNECT_TIMEOUT = TIME_MS2I(30000),
	};

	typedef enum {
		STATE_IDLE,
		STATE_WAITING_FOR_RESPONSE,
		STATE_DATA_MODE
	} AtParserState;

	typedef struct {
		uint8_t linkId;
		input_queue_t iqueue;
		uint8_t ibuffer[CHANNEL_BUFFERS_SIZE];
		ChannelType type;
		char remoteAddr[IP_ADDRESS_SIZE];
		uint16_t remotePort;
		uint16_t localPort;
	} Channels;

	/**
	 * @brief	Response entry type.
	 */
	typedef struct {
		const char* name;	/**< @brief Message name. */
		msg_t msg;			/**< @brief Message value. */
	} RespMsg;

	/**
	 * @brief	Supported response entries.
	 */
	static const RespMsg m_respMsg[];

	static ESP m_instance;
	static THD_WORKING_AREA(m_thread_ctx, 512);
	SerialDriver* m_serial;
	ioline_t m_rstPin;
	thread_t* m_thread;
	event_source_t m_event;
	AtParserState m_atParserState;
	mutex_t m_mutex;
	char m_buffer[BUFFER_SIZE];
	char *m_pbuffer;
	uint8_t m_linkId;
	size_t m_length;
	Channels m_channels[NUM_CHANNELS];

	ESP();
	ESP& operator=(const ESP &other) = delete;
	ESP(const ESP &other) = delete;

	static void run(void *args);
	void run();

	/**
	 * @brief	Reads line with timeout.
	 * @details	Drives the state machine according to received characters.
	 * 			Receives complete lines ended with '\n' and discards '\r'
	 * 			to simplify further line handling. Also receives channel
	 * 			data which are put to input queue so the upper layer can
	 * 			handle it accordingly. Some of the unsolicited messages
	 * 			are handled as events and broadcasted to listeners.
	 *
	 * @note	Responses are in form <CR><LF>RESPONSE<CR><LF>, unsolicited
	 * 			messages in form MESSAGE<CR><LF>, channel data in form
	 * 			<CR><LF>+IPD,... not ended by <CR><LF>.
	 *
	 * @param[in] mask		the response flags set to be ORed
	 * @param[in] cb		pointer to a callback function, can be NULL
	 * @param[in] timeout	the number of ticks before the operation timeouts,
	 * 						the following special values are allowed:
	 * 						- @a TIME_IMMEDIATE immediate timeout.
	 * 						- @a TIME_INFINITE no timeout.
	 * 						.
	 *
	 * @return				Response message.
	 * @retval MSG_TIMEOUT	if the specified time expired.
	 * @retval MSG_RESET	if the channel associated queue (if any) has been reset.
	 * @retval MSG_LINE		if a new line was received that is not a response message
	 */
	msg_t readline(msg_t mask, std::function<void()> cb, sysinterval_t timeout);

	/**
	 * @brief	Handles ESP8266 command.
	 *
	 * @param[in] mask		the response flags set to be ORed
	 * @param[in] cb		pointer to a callback function, can be NULL
	 * @param[in] timeout	the number of ticks before the operation timeouts,
	 * 						the following special values are allowed:
	 * 						- @a TIME_IMMEDIATE immediate timeout.
	 * 						- @a TIME_INFINITE no timeout.
	 * 						.
	 * @param[in] fmt		formatting string with AT command
	 *
	 * @return				Response message.
	 * @retval MSG_TIMEOUT	if the specified time expired.
	 * @retval MSG_RESET	if the channel associated queue (if any) has been reset.
	 */
	msg_t command(msg_t resp, std::function<void()> cb, sysinterval_t timeout, const char *fmt, ...);

	/**
	 * @brief	Configures the multiple connections mode.
	 *
	 * @param[in] enable	flag indicating if multiple connections mode is enabled or not
	 *
	 * @return 				Operation result.
	 * @retval MSG_OK		on success.
	 * @retval MSG_ERROR	if an error occurred.
	 * @retval MSG_TIMEOUT	if the response @a RESP_TIMEOUT time expired.
	 * @retval MSG_RESET	if the channel associated queue (if any) has been reset.
	 */
	msg_t ipEnableMuxMode(bool enable);

	/**
	 * @brief	Sets the size of SSL buffer.
	 *
	 * @param[in] size		the size of the SSL buffer; range of value: [2048, 4096]
	 *
	 * @return 				Operation result.
	 * @retval MSG_OK		on success.
	 * @retval MSG_ERROR	if an error occurred.
	 * @retval MSG_TIMEOUT	if the response @a RESP_TIMEOUT time expired.
	 * @retval MSG_RESET	if the channel associated queue (if any) has been reset.
	 */
	msg_t ipSetSSLBufferSize(uint16_t size = 4096);

	/**
	 * @brief	Send data header.
	 *
	 * @param[in] linkId	ID of the connection (0~4), for multiple connections
	 * @param[in] n			the maximum amount of data to be transferred
	 *
	 * @return				Operation result.
	 */
	bool ipSendDataHeader(uint8_t linkId, size_t n);

public:
	/** @brief	Returns static instance of power manager class. */
	static ESP& instance() { return m_instance; }

	/** @brief	Returns event source. */
	event_source_t& event() { return m_event; }

	/**
	 * @brief	Configures the ESP8266.
	 */
	void init(SerialDriver& serial, ioline_t rstPin);

	/**
	 * @brief	Activates the ESP8266.
	 * @details	Creates the background handling thread, disables echoing,
	 * 			sets multiplexing mode and SSL buffer size.
	 *
	 * @return 				Operation result.
	 * @retval MSG_OK		on success.
	 * @retval MSG_ERROR	if an error occurred.
	 * @retval MSG_TIMEOUT	if the response @a RESP_TIMEOUT time expired.
	 * @retval MSG_RESET	if the channel associated queue (if any) has been reset.
	 */
	msg_t start();

	/**
	 * @brief	Deactivates the ESP8266.
	 * @note	The call to stop the serial driver interrupts the never-ending thread
	 * 			loop and allows the thread to terminate in case of waiting in sleep.
	 *
	 * @return 				Operation result.
	 * @retval MSG_OK		on success.
	 */
	msg_t stop();

	/**
	 * @brief	Restarts the module and waits until it is ready.
	 *
	 * @return 				Operation result.
	 * @retval MSG_OK		on success.
	 * @retval MSG_ERROR	if an error occurred.
	 * @retval MSG_TIMEOUT	if the response @a RESP_TIMEOUT time expired.
	 * @retval MSG_RESET	if the channel associated queue (if any) has been reset.
	 */
	msg_t reset();

	/**
	 * @brief	Enables/Disables AT commands echoing.
	 *
	 * @param[in] enable	flag indicating if echoing is enabled or not
	 *
	 * @return 				Operation result.
	 * @retval MSG_OK		on success.
	 * @retval MSG_ERROR	if an error occurred.
	 * @retval MSG_TIMEOUT	if the response @a RESP_TIMEOUT time expired.
	 * @retval MSG_RESET	if the channel associated queue (if any) has been reset.
	 */
	msg_t enableEcho(bool enable);

	msg_t enterDeepSleep(uint32_t timeout);

	/**
	 * @brief	Sets the WiFi mode.
	 *
	 * @param[in] mode			WiFi @p WifiMode mode
	 * @param[in] defaultCfg	flag indicating if store configuration in Flash
	 *
	 * @return 				Operation result.
	 * @retval MSG_OK		on success.
	 * @retval MSG_ERROR	if an error occurred.
	 * @retval MSG_TIMEOUT	if the response @a RESP_TIMEOUT time expired.
	 * @retval MSG_RESET	if the channel associated queue (if any) has been reset.
	 */
	msg_t wifiSetMode(WifiMode mode, bool defaultCfg = false);

	/**
	 * @brief	Connects to an access point.
	 *
	 * @param[in] ssid			the SSID of the target AP
	 * @param[in] pwd			password, max 64-byte ASCII
	 * @param[in] defaultCfg	flag indicating if store configuration in Flash
	 *
	 * @return 						Operation result.
	 * @retval MSG_OK				on success.
	 * @retval MSG_ERROR			if an error occurred.
	 * @retval MSG_FAIL				on failure.
	 * @retval MSG_CONN_TIMEOUT		on connection timeout.
	 * @retval MSG_CONN_INVALID_PWD	on wrong password.
	 * @retval MSG_CONN_INVALID_AP	if target access point cannot be found.
	 * @retval MSG_CONN_FAILED		on connection failure.
	 * @retval MSG_TIMEOUT			if the response @a CONNECT_TIMEOUT time expired.
	 * @retval MSG_RESET			if the channel associated queue (if any) has been reset.
	 */
	msg_t wifiConnect(const char* ssid, const char* pwd, bool defaultCfg = false);

	/**
	 * @brief	Disconnects from the AP.
	 *
	 * @return 				Operation result.
	 * @retval MSG_OK		on success.
	 * @retval MSG_ERROR	if an error occurred.
	 * @retval MSG_TIMEOUT	if the response @a RESP_TIMEOUT time expired.
	 * @retval MSG_RESET	if the channel associated queue (if any) has been reset.
	 */
	msg_t wifiDisconnect();

	/**
	 * @brief	Connects to the AP automatically on power-up by default.
	 *
	 * @param[in] enable	flag indicating if auto connect on power-up
	 *
	 * @return 				Operation result.
	 * @retval MSG_OK		on success.
	 * @retval MSG_ERROR	if an error occurred.
	 * @retval MSG_TIMEOUT	if the response @a RESP_TIMEOUT time expired.
	 * @retval MSG_RESET	if the channel associated queue (if any) has been reset.
	 */
	msg_t wifiAutoConnect(bool enable);

	/**
	 * @brief	Configures the ESP8266 SoftAP.
	 *
	 * @param[in] ssid			string parameter, SSID of AP
	 * @param[in] pwd			string parameter; length of password: 8 ~ 64 bytes ASCII
	 * @param[in] chn			channel ID
	 * @param[in] enc			encryption method @p esp8266_wifi_enc_t
	 * @param[in] maxConn		maximum number of stations to which ESP8266 SoftAP can be
	 * 							connected, within the range of [1, 8]
	 * @param[in] hidden		flag indicating if the SSID is broadcasted or not
	 * @param[in] defaultCfg	flag indicating if store configuration in Flash
	 *
	 * @return 				Operation result.
	 * @retval MSG_OK		on success.
	 * @retval MSG_ERROR	if an error occurred.
	 * @retval MSG_TIMEOUT	if the response @a RESP_TIMEOUT time expired.
	 * @retval MSG_RESET	if the channel associated queue (if any) has been reset.
	 */
	msg_t wifiSetAP(const char *ssid, const char *pwd, uint8_t chn, WifiEnc enc, uint8_t maxConn, bool hidden,  bool defaultCfg = false);

	/**
	 * @brief	Enables/Disables DHCP.
	 *
	 * @param[in] mode			WiFi @p WifiMode mode
	 * @param[in] enable		flag indicating if DHCP is enabled or not
	 * @param[in] defaultCfg	flag indicating if store configuration in Flash
	 *
	 * @return 				Operation result.
	 * @retval MSG_OK		on success.
	 * @retval MSG_ERROR	if an error occurred.
	 * @retval MSG_TIMEOUT	if the response @a RESP_TIMEOUT time expired.
	 * @retval MSG_RESET	if the channel associated queue (if any) has been reset.
	 */
	msg_t wifiEnableDHCP(WifiMode mode, bool enable, bool defaultCfg = false);

	/**
	 * @brief	Sets the IP address allocated by ESP8266 SoftAP DHCP.
	 *
	 * @param[in] leaseTime		lease time; unit: minute; range [1, 2880]
	 * @param[in] startIp		start IP of the IP range that can be obtained
	 * 							from ESP8266 SoftAP DHCP server
	 * @param[in] endIp			end IP of the IP range that can be obtained
	 * 							from ESP8266 SoftAP DHCP server
	 * @param[in] defaultCfg	flag indicating if store configuration in Flash
	 *
	 * @return 					Operation result.
	 * @retval MSG_OK			on success.
	 * @retval MSG_ERROR		if an error occurred.
	 * @retval MSG_TIMEOUT		if the response @a RESP_TIMEOUT time expired.
	 * @retval MSG_RESET		if the channel associated queue (if any) has been reset.
	 */
	msg_t wifiSetDHCPIpRange(uint16_t leaseTime, const char *startIp, const char *endIp, bool defaultCfg = false);

	/**
	 * @brief	Sets the default IP address range allocated by ESP8266 SoftAP DHCP.
	 *
	 * @param[in] defaultCfg	flag indicating if store configuration in Flash
	 *
	 * @return 				Operation result.
	 * @retval MSG_OK		on success.
	 * @retval MSG_ERROR	if an error occurred.
	 * @retval MSG_TIMEOUT	if the response @a RESP_TIMEOUT time expired.
	 * @retval MSG_RESET	if the channel associated queue (if any) has been reset.
	 */
	msg_t wifiSetDHCPIpRange(bool defaultCfg = false);

	/**
	 * @brief	Gets the connection status.
	 *
	 * @param[out] status	pointer to the @p Status object
	 *
	 * @return 				Operation result.
	 * @retval MSG_OK		on success.
	 * @retval MSG_ERROR	if an error occurred.
	 * @retval MSG_TIMEOUT	if the response @a RESP_TIMEOUT time expired.
	 * @retval MSG_RESET	if the channel associated queue (if any) has been reset.
	 */
	msg_t ipGetStatus(Status& status);

	/**
	 * @brief	Establishes TCP or SSL connection.
	 *
	 * @param[in] linkID		ID of network connection (0~4), used for multiple connections
	 * @param[in] host			string parameter indicating the remote IP address
	 * @param[in] remotePort	the remote port number
	 * @param[in] keepAlive		detection time interval when TCP is kept alive;
	 * 							- 0: disable TCP keep-alive
	 * 							- 1	~ 7200: detection time interval; unit: second (s)
	 * @param[in] secure		flag indicating if SSL or TCP should be used
	 *
	 * @return 					Operation result.
	 * @retval MSG_OK			on success.
	 * @retval MSG_ERROR		if an error occurred.
	 * @retval MSG_TIMEOUT		if the response @a RESP_TIMEOUT time expired.
	 * @retval MSG_RESET		if the channel associated queue (if any) has been reset.
	 */
	msg_t ipConnect(uint8_t linkId, const char *host, uint16_t remotePort, uint16_t keepAlive, bool secure);

	/**
	 * @brief	Establishes UDP connection.
	 *
	 * @param[in] linkID		ID of network connection (0~4), used for multiple connections
	 * @param[in] host			string parameter indicating the remote IP address
	 * @param[in] remotePort	the remote port number
	 * @param[in] localPort		the local port number
	 * @param[in] mode			UDP @p UdpMode mode
	 *
	 * @return 					Operation result.
	 * @retval MSG_OK			on success.
	 * @retval MSG_ERROR		if an error occurred.
	 * @retval MSG_TIMEOUT		if the response @a RESP_TIMEOUT time expired.
	 * @retval MSG_RESET		if the channel associated queue (if any) has been reset.
	 */
	msg_t ipConnect(uint8_t linkId, const char *host, uint16_t remotePort, uint16_t localPort, UdpMode mode);

	/**
	 * @brief	Closes the TCP/UDP/SSL connection.
	 *
	 * @param[in] linkID	ID of the connection to be closed, when ID
							is 5, all connections will be closed.
	 *
	 * @return 				Operation result.
	 * @retval MSG_OK		on success.
	 * @retval MSG_ERROR	if an error occurred.
	 * @retval MSG_TIMEOUT	if the response @a RESP_TIMEOUT time expired.
	 * @retval MSG_RESET	if the channel associated queue (if any) has been reset.
	 */
	msg_t ipDisconnect(uint8_t linkId);

	/**
	 * @brief	Sends data.
	 *
	 * @param[in] linkId	ID of the connection (0~4), for multiple connections
	 * @param[in] buf		pointer to the data buffer
	 * @param[in] n			the maximum amount of data to be transferred
	 *
	 * @return 				The number of bytes transferred.
	 */
	size_t ipSendData(uint8_t linkId, const uint8_t *buf, size_t n);

	/**
	 * @brief	Sends data using iterator.
	 *
	 * @param[in] linkId	ID of the connection (0~4), for multiple connections
	 * @param[in] first		input iterator first element
	 * @param[in] last		input iterator last element
	 *
	 * @return				The current iterator position.
	 */
	template <typename Iterator>
	Iterator ipSendData(uint8_t linkId, const Iterator& first, const Iterator& last) {
		Iterator it = first;
		size_t n = std::distance(first, last);
		msg_t ret;

		chMtxLock(&m_mutex);

		while (n > 0) {
			// send header
			size_t bytesToSend = (n > CHN_MAX_BUFFER_SIZE) ? CHN_MAX_BUFFER_SIZE : n;
			if (!ipSendDataHeader(linkId, bytesToSend))
				break;

			// send data
			while (bytesToSend-- && sdPut(m_serial, *it++) != MSG_RESET);

			do {
				ret = readline((1 << MSG_OK) | (1 << MSG_ERROR), nullptr, RESP_TIMEOUT);
			} while (ret == MSG_LINE);

			if (ret != MSG_OK)
				break;

			n = std::distance(it, last);
		}

		chMtxUnlock(&m_mutex);

		return it;
	}

	/**
	 * @brief	Reads data with timeout.
	 *
	 * @param[in] linkId	ID of the connection (0~4), for multiple connections
	 * @param[in] buf		pointer to the data buffer
	 * @param[in] n			the maximum amount of data to be transferred
	 * @param[in] timeout	the number of ticks before the operation timeouts,
	 * 						the following special values are allowed:
	 * 						- @a TIME_IMMEDIATE immediate timeout.
	 * 						- @a TIME_INFINITE no timeout.
	 * 						.
	 * @return 				The number of bytes transferred.
	 */
	size_t ipReceiveData(uint8_t linkId, uint8_t *buf, size_t n, sysinterval_t timeout);

	/**
	 * @brief	Reads byte with timeout.
	 *
	 * @param[in] linkId	ID of the connection (0~4), for multiple connections
	 * @param[in] timeout	the number of ticks before the operation timeouts,
	 * 						the following special values are allowed:
	 * 						- @a TIME_IMMEDIATE immediate timeout.
	 * 						- @a TIME_INFINITE no timeout.
	 * 						.
	 * @return				A byte value from the queue.
	 * @retval MSG_TIMEOUT	if the specified time expired.
	 * @retval MSG_RESET	if the queue has been reset.
	 */
	msg_t ipReceiveData(uint8_t linkId, sysinterval_t timeout);

	/**
	 * @brief	Creates TCP server.
	 *
	 * @param[in] localPort		local port number
	 *
	 * @return 				Operation result.
	 * @retval MSG_OK		on success.
	 * @retval MSG_ERROR	if an error occurred.
	 * @retval MSG_TIMEOUT	if the response @a RESP_TIMEOUT time expired.
	 * @retval MSG_RESET	if the channel associated queue (if any) has been reset.
	 */
	msg_t ipCreateServer(uint16_t localPort);

	/**
	 * @brief	Deletes TCP server.
	 *
	 * @return 				Operation result.
	 * @retval MSG_OK		on success.
	 * @retval MSG_ERROR	if an error occurred.
	 * @retval MSG_TIMEOUT	if the response @a RESP_TIMEOUT time expired.
	 * @retval MSG_RESET	if the channel associated queue (if any) has been reset.
	 */
	msg_t ipDeleteServer();

	/**
	 * @brief	Gets the Local IP Address.
	 *
	 * @param[in] apIPAddr		IP address of the ESP8266 SoftAP
	 * @param[in] apMACAddr		MAC address of the ESP8266 SoftAP
	 * @param[in] staIPAddr		IP address of the ESP8266 Station
	 * @param[in] staMACAddr	MAC address of the ESP8266 Station
	 *
	 * @return 				Operation result.
	 * @retval MSG_OK		on success.
	 * @retval MSG_ERROR	if an error occurred.
	 * @retval MSG_TIMEOUT	if the response @a RESP_TIMEOUT time expired.
	 * @retval MSG_RESET	if the channel associated queue (if any) has been reset.
	 */
	msg_t ipGetAddres(std::string& apIPAddr, std::string& apMACAddr, std::string& staIPAddr, std::string& staMACAddr);

	/**
	 * @brief	Sets the maximum connections allowed by server.
	 *
	 * @param[in] maxConn	the maximum number of clients allowed to connect to
	 * 						the TCP or SSL server, range: [1, 5]
	 *
	 * @return 				Operation result.
	 * @retval MSG_OK		on success.
	 * @retval MSG_ERROR	if an error occurred.
	 * @retval MSG_TIMEOUT	if the response @a RESP_TIMEOUT time expired.
	 * @retval MSG_RESET	if the channel associated queue (if any) has been reset.
	 */
	msg_t ipSetServerMaxConnections(uint8_t maxConn);
};

#endif /* ESP_HPP_ */
