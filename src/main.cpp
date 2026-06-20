/**
 * @file main.cpp
 * @brief ESP-NOW Slave Transceiver – responds to master discovery and polls.
 *
 * The slave listens for PKT_DISCOVER frames broadcast by the master.  Upon
 * receiving one it registers the master's MAC as a unicast peer and replies
 * with PKT_DISCOVER_RESP carrying its own sensor data snapshot.  It also
 * handles PKT_POLL requests, responding with an up-to-date PKT_POLL_RESP.
 *
 * Node ID is derived automatically from the last byte of the device's own
 * MAC address, so each slave is self-identifying without any manual config.
 *
 * The sensor_data_t structure is a placeholder that mirrors the master's
 * definition.  Replace the fields to match the actual home-monitoring
 * signals once the hardware design is finalised.
 *
 * Protocol packet types (must stay in sync with master firmware):
 *   PKT_DISCOVER      0x01  master → broadcast
 *   PKT_DISCOVER_RESP 0x02  slave  → master
 *   PKT_POLL          0x03  master → slave
 *   PKT_POLL_RESP     0x04  slave  → master
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <stdint.h>
#include <string.h>
#include "gprintf.h"
#include "soc/rtc_cntl_reg.h"

// ============================================================================
// Constants
// ============================================================================

/** @brief Length of a MAC address in bytes. */
#define MAC_ADDR_LEN              6U

/** @brief Depth of the inter-task receive queue (callback → loop). */
#define RX_QUEUE_DEPTH            8U

/** @brief Depth of the outbound response queue (loop → esp_now_send). */
#define TX_QUEUE_DEPTH            4U

/** @brief How often the placeholder sensor data is refreshed (ms). */
#define SENSOR_UPDATE_INTERVAL_MS 500U

/** @brief Retry delay before re-attempting initialisation after an error (ms). */
#define INIT_RETRY_DELAY_MS       5000U

/** @brief Minimum acceptable received payload size. */
#define MIN_PACKET_LEN            ((int32_t)sizeof(espnow_packet_t))

/** @brief SSID of the master's soft-AP – used to discover its channel. */
#define MASTER_AP_SSID            "ESP32-HomeMon"

/** @brief Delay after WiFi.mode() before starting scan (ms). */
#define WIFI_RADIO_SETTLE_MS      300U

/** @brief Timeout waiting for the channel scan to complete (ms). */
#define CHANNEL_SCAN_TIMEOUT_MS   10000U

/** @brief Fallback ESP-NOW channel when master AP is not found in scan. */
#define DEFAULT_ESPNOW_CHANNEL    1U

/** @brief LED blink interval while scanning for master (ms). */
#define LED_SCAN_BLINK_MS         1000U

/** @brief LED blink interval once connected to a master peer (ms). */
#define LED_CONN_BLINK_MS         250U

/** @brief If no master contact within this window (ms), re-scan for its channel. */
#define RESCAN_INTERVAL_MS      30000U

// ============================================================================
// Type definitions  (must stay byte-for-byte identical to the master's types)
// ============================================================================

/**
 * @brief Packet type field values.
 */
typedef enum {
    PKT_DISCOVER      = 0x01U, /**< Master → broadcast: find all slaves.       */
    PKT_DISCOVER_RESP = 0x02U, /**< Slave  → master:    identify self.         */
    PKT_POLL          = 0x03U, /**< Master → slave:     request sensor data.   */
    PKT_POLL_RESP     = 0x04U, /**< Slave  → master:    current sensor data.   */
} pkt_type_t;

/**
 * @brief Placeholder sensor payload – replace fields for your application.
 */
typedef struct {
    uint8_t  nodeId;        /**< This slave's node identifier.                 */
    uint16_t analogValue;   /**< Example: raw 12-bit ADC reading.              */
    uint8_t  digitalInputs; /**< Example: bitmask of up to 8 digital inputs.  */
    uint32_t uptimeSec;     /**< Uptime in seconds since last boot.            */
} sensor_data_t;

/**
 * @brief Common packet header.
 */
typedef struct {
    uint8_t  pktType;      /**< One of pkt_type_t.                            */
    uint8_t  senderId;     /**< Sender's node ID.                             */
    uint32_t timestampMs;  /**< Sender's millis() at transmit time.           */
} pkt_header_t;

/**
 * @brief Complete over-the-air packet: header plus sensor payload.
 */
typedef struct {
    pkt_header_t  header; /**< Routing and protocol information.              */
    sensor_data_t data;   /**< Sensor payload.                               */
} espnow_packet_t;

/**
 * @brief Item stored in the receive ring-buffer (callback → loop).
 */
typedef struct {
    uint8_t         srcMac[MAC_ADDR_LEN]; /**< Source MAC from the callback.  */
    espnow_packet_t pkt;                  /**< Full received packet.          */
} rx_item_t;

/**
 * @brief Item stored in the transmit queue (loop → esp_now_send).
 */
typedef struct {
    uint8_t    destMac[MAC_ADDR_LEN]; /**< Unicast destination MAC.          */
    pkt_type_t pktType;               /**< Response packet type.             */
} tx_request_t;

/**
 * @brief Slave state-machine states.
 */
typedef enum {
    SLAVE_STATE_WIFI_INIT,    /**< Set WiFi STA mode; start radio-settle timer. */
    SLAVE_STATE_CHANNEL_SCAN, /**< Start async scan for master soft-AP.         */
    SLAVE_STATE_CHANNEL_WAIT, /**< Wait for scan; extract master channel.       */
    SLAVE_STATE_ESPNOW_INIT,  /**< Initialise ESP-NOW on master's channel.      */
    SLAVE_STATE_LISTENING,    /**< Operational: handle requests, update data.   */
    SLAVE_STATE_RESCAN,       /**< No master seen; deinit ESP-NOW and re-scan.  */
    SLAVE_STATE_ERROR,        /**< Unrecoverable error – retry after delay.     */
} slave_state_t;

// ============================================================================
// Static state
// ============================================================================

/** @brief This slave's auto-assigned node ID (set from MAC at init). */
static uint8_t       slaveNodeId       = 0U;

/** @brief Current placeholder sensor readings. */
static sensor_data_t sensorData;

/** @brief millis() timestamp of the last sensor data refresh. */
static uint32_t      sensorUpdateMs    = 0U;

/** @brief Current slave state. */
static slave_state_t slaveState        = SLAVE_STATE_WIFI_INIT;

/** @brief WiFi channel on which the master soft-AP was found. */
static uint8_t       masterChannel     = DEFAULT_ESPNOW_CHANNEL;

/** @brief millis() recorded when the current state was entered. */
static uint32_t      stateTimestampMs  = 0U;

/** @brief True once a master peer has been successfully registered. */
static bool          masterConnected   = false;

/** @brief millis() of the last onboard LED toggle. */
static uint32_t      ledToggleMs       = 0U;

/** @brief millis() when SLAVE_STATE_LISTENING was entered or master last seen. */
static uint32_t      rescanTimerMs     = 0U;

// ---------------------------------------------------------------------------
// Receive queue – written by the ESP-NOW WiFi-task callback, read by loop().
// ---------------------------------------------------------------------------

/** @brief Ring-buffer of received packets pending main-loop processing. */
static volatile rx_item_t rxQueue[RX_QUEUE_DEPTH];

/** @brief Write index for rxQueue (updated by ESP-NOW callback). */
static volatile uint8_t   rxHead = 0U;

/** @brief Read index for rxQueue (updated by main loop). */
static volatile uint8_t   rxTail = 0U;

// ---------------------------------------------------------------------------
// Transmit queue – populated by processRxQueue(), drained by sendPending().
// ---------------------------------------------------------------------------

/** @brief Queue of outbound response requests. */
static tx_request_t txQueue[TX_QUEUE_DEPTH];

/** @brief Write index for txQueue. */
static uint8_t      txHead = 0U;

/** @brief Read index for txQueue. */
static uint8_t      txTail = 0U;

/** @brief millis() before which the TX queue is held (collision-avoidance backoff). */
static uint32_t     txHoldUntilMs = 0U;

// ============================================================================
// Forward declarations
// ============================================================================

static void    updateLed          (void);
static void    slaveProcess       (void);
static void    processRxQueue     (void);
static void    sendPending        (void);
static void    handleDiscover     (const uint8_t *masterMac);
static void    handlePoll         (const uint8_t *masterMac);
static void    enqueueTx          (const uint8_t *destMac, pkt_type_t pktType);
static void    sendPacket         (const uint8_t *destMac, pkt_type_t pktType);
static void    registerPeer       (const uint8_t *macAddr);
static void    updateSensorData   (void);
static bool    initEspNowSlave    (void);
static void    macToString        (const uint8_t *mac, char *buf);

// ============================================================================
// ESP-NOW callbacks  (run in the WiFi task – not the Arduino loop task)
// ============================================================================

/**
 * @brief Invoked by ESP-NOW after each transmission attempt.
 * @param[in] macAddr   Destination MAC of the transmission.
 * @param[in] status    Delivery status.
 */
static void onDataSent(const uint8_t *macAddr, esp_now_send_status_t status)
{
    (void)macAddr;
    if (status != ESP_NOW_SEND_SUCCESS)
    {
        gprintf(gDBG, "[SLAVE] TX delivery failed\r\n");
    }
}

/**
 * @brief Invoked by ESP-NOW when a packet is received.
 *
 * Runs in the WiFi task context.  Enqueues the frame for processing in
 * loop() without blocking the WiFi stack.
 *
 * @param[in] srcMac    6-byte source MAC address.
 * @param[in] data      Raw payload bytes.
 * @param[in] dataLen   Payload length.  Type int mandated by esp_now_recv_cb_t.
 */
static void onDataRecv(const uint8_t *srcMac,
                       const uint8_t *data,
                       int            dataLen) /* int: required by esp_now_recv_cb_t */
{
    if ((srcMac == NULL) || (data == NULL))
        return;
    if ((int32_t)dataLen < MIN_PACKET_LEN)
        return;

    uint8_t nextHead = (rxHead + 1U) % RX_QUEUE_DEPTH;
    if (nextHead == rxTail)
        return; /* Queue full – drop; main loop is behind. */

    memcpy((void *)rxQueue[rxHead].srcMac, srcMac, MAC_ADDR_LEN);
    memcpy((void *)&rxQueue[rxHead].pkt,   data,   sizeof(espnow_packet_t));
    rxHead = nextHead;
}

// ============================================================================
// Peer management
// ============================================================================

/**
 * @brief Registers a MAC address with the ESP-NOW driver as a unicast peer.
 *
 * No-ops silently if the peer is already registered.
 *
 * @param[in] macAddr   6-byte MAC address to register.
 */
static void registerPeer(const uint8_t *macAddr)
{
    if (esp_now_is_peer_exist(macAddr))
    {
        masterConnected = true;
        return;
    }

    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, macAddr, MAC_ADDR_LEN);
    peerInfo.channel = 0U;
    peerInfo.encrypt = false;

    esp_err_t result = esp_now_add_peer(&peerInfo);
    if (result != ESP_OK)
    {
        char macStr[18];
        macToString(macAddr, macStr);
        gprintf(gDBG, "[SLAVE] esp_now_add_peer(%s) err=%d\r\n",
                macStr, (int32_t)result);
    }
    else
    {
        char macStr[18];
        macToString(macAddr, macStr);
        gprintf(gDBG, "[SLAVE] Registered master peer: %s\r\n", macStr);
        masterConnected = true;
    }
}

// ============================================================================
// Request handlers
// ============================================================================

/**
 * @brief Handles an incoming PKT_DISCOVER from the master.
 *
 * Registers the master as a unicast peer and enqueues a PKT_DISCOVER_RESP.
 *
 * @param[in] masterMac  6-byte source MAC of the master.
 */
static void handleDiscover(const uint8_t *masterMac)
{
    registerPeer(masterMac);
    enqueueTx(masterMac, PKT_DISCOVER_RESP);
    gprintf(gDBG, "[SLAVE] PKT_DISCOVER received – queued DISCOVER_RESP\r\n");
}

/**
 * @brief Handles an incoming PKT_POLL from the master.
 *
 * Registers the master as a peer if not already known, then enqueues a
 * PKT_POLL_RESP carrying the latest sensor snapshot.
 *
 * @param[in] masterMac  6-byte source MAC of the master.
 */
static void handlePoll(const uint8_t *masterMac)
{
    registerPeer(masterMac);
    enqueueTx(masterMac, PKT_POLL_RESP);
}

// ============================================================================
// Transmit queue
// ============================================================================

/**
 * @brief Adds a response request to the transmit queue.
 *
 * Silently drops the request when the queue is full.
 *
 * @param[in] destMac   6-byte destination MAC address.
 * @param[in] pktType   Response packet type to send.
 */
static void enqueueTx(const uint8_t *destMac, pkt_type_t pktType)
{
    uint8_t nextHead = (txHead + 1U) % TX_QUEUE_DEPTH;
    if (nextHead == txTail)
    {
        gprintf(gDBG, "[SLAVE] TX queue full – response dropped\r\n");
        return;
    }
    memcpy(txQueue[txHead].destMac, destMac, MAC_ADDR_LEN);
    txQueue[txHead].pktType = pktType;
    txHead = nextHead;

    /* DISCOVER_RESP: random 0-199 ms backoff so multiple slaves that hear the
     * same broadcast don't transmit simultaneously (hidden-node collision). */
    if (pktType == PKT_DISCOVER_RESP)
    {
        txHoldUntilMs = millis() + (esp_random() % 200U);
    }
}

/**
 * @brief Drains the transmit queue by sending each pending response packet.
 */
static void sendPending(void)
{
    if (millis() < txHoldUntilMs)
        return;

    while (txTail != txHead)
    {
        sendPacket(txQueue[txTail].destMac, txQueue[txTail].pktType);
        txTail = (txTail + 1U) % TX_QUEUE_DEPTH;
    }
}

/**
 * @brief Builds and transmits an ESP-NOW response packet to the given destination.
 *
 * Populates the header with the slave's node ID and current millis(), and
 * fills the data field with the latest sensor snapshot.
 *
 * @param[in] destMac   6-byte destination MAC address.
 * @param[in] pktType   Protocol packet type (PKT_DISCOVER_RESP or PKT_POLL_RESP).
 */
static void sendPacket(const uint8_t *destMac, pkt_type_t pktType)
{
    espnow_packet_t outPkt;
    memset(&outPkt, 0, sizeof(outPkt));
    outPkt.header.pktType     = (uint8_t)pktType;
    outPkt.header.senderId    = slaveNodeId;
    outPkt.header.timestampMs = millis();
    memcpy(&outPkt.data, &sensorData, sizeof(sensor_data_t));

    esp_err_t result = esp_now_send(destMac,
                                    (const uint8_t *)&outPkt,
                                    sizeof(outPkt));
    if (result != ESP_OK)
    {
        gprintf(gDBG, "[SLAVE] esp_now_send err=%d\r\n", (int32_t)result);
    }
}

// ============================================================================
// Receive queue processing
// ============================================================================

/**
 * @brief Drains rxQueue and dispatches each received packet to its handler.
 *
 * Only PKT_DISCOVER and PKT_POLL are acted upon; all other types are ignored.
 */
static void processRxQueue(void)
{
    while (rxTail != rxHead)
    {
        rx_item_t item;
        memcpy(item.srcMac, (const void *)rxQueue[rxTail].srcMac, MAC_ADDR_LEN);
        memcpy(&item.pkt,   (const void *)&rxQueue[rxTail].pkt,   sizeof(espnow_packet_t));
        rxTail = (rxTail + 1U) % RX_QUEUE_DEPTH;

        switch ((pkt_type_t)item.pkt.header.pktType)
        {
        case PKT_DISCOVER:
            handleDiscover(item.srcMac);
            break;

        case PKT_POLL:
            handlePoll(item.srcMac);
            break;

        default:
            break;
        }
    }
}

// ============================================================================
// Sensor data
// ============================================================================

/**
 * @brief Refreshes the placeholder sensor_data_t at SENSOR_UPDATE_INTERVAL_MS.
 *
 * Replace this function body with real ADC reads, GPIO sampling, etc. when
 * the actual sensor hardware is connected.
 */
static void updateSensorData(void)
{
    uint32_t nowMs = millis();
    if ((nowMs - sensorUpdateMs) < SENSOR_UPDATE_INTERVAL_MS)
        return;

    sensorUpdateMs          = nowMs;
    sensorData.nodeId       = slaveNodeId;
    sensorData.uptimeSec    = nowMs / 1000U;
    sensorData.analogValue  = (uint16_t)(nowMs / 100U) & 0x0FFFU; /* 12-bit wrap */
    sensorData.digitalInputs = 0x00U; /* placeholder: all inputs low */
}

// ============================================================================
// Initialisation
// ============================================================================

/**
 * @brief Initialises ESP-NOW and derives the slave's node ID from its MAC.
 *
 * WiFi mode and channel must already be configured before calling this.
 * The node ID is set to the least-significant byte of the device MAC,
 * guaranteeing uniqueness across all slaves without manual configuration.
 *
 * @return true on success, false if any ESP-NOW step failed.
 */
static bool initEspNowSlave(void)
{
    uint8_t ownMac[MAC_ADDR_LEN];
    WiFi.macAddress(ownMac);
    slaveNodeId = ownMac[MAC_ADDR_LEN - 1U];

    if (esp_now_init() != ESP_OK)
    {
        gprintf(gDBG, "[SLAVE] esp_now_init() failed\r\n");
        return false;
    }

    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    char macStr[18];
    macToString(ownMac, macStr);
    gprintf(gDBG, "[SLAVE] ESP-NOW ready on ch%u. MAC=%s  NodeID=%u\r\n",
            (uint32_t)masterChannel, macStr, (uint32_t)slaveNodeId);
    return true;
}

// ============================================================================
// Utility
// ============================================================================

/**
 * @brief Formats six raw MAC bytes into the "XX:XX:XX:XX:XX:XX\0" string.
 * @param[in]  mac  6-byte MAC address.
 * @param[out] buf  Output buffer – caller must provide at least 18 bytes.
 */
static void macToString(const uint8_t *mac, char *buf)
{
    static const char hexChars[] = "0123456789ABCDEF";
    for (uint8_t i = 0U; i < MAC_ADDR_LEN; i++)
    {
        buf[(uint8_t)(i * 3U)]      = hexChars[(mac[i] >> 4U) & 0x0FU];
        buf[(uint8_t)(i * 3U + 1U)] = hexChars[mac[i] & 0x0FU];
        buf[(uint8_t)(i * 3U + 2U)] = (i < (MAC_ADDR_LEN - 1U)) ? ':' : '\0';
    }
    buf[17] = '\0';
}

// ============================================================================
// LED status indicator
// ============================================================================

/**
 * @brief Toggles the onboard LED at a rate that reflects connection state.
 *
 * Blinks at LED_SCAN_BLINK_MS while no master peer is known, and switches to
 * the faster LED_CONN_BLINK_MS cadence once the master has been registered.
 */
static void updateLed(void)
{
    uint32_t nowMs    = millis();
    uint32_t interval = masterConnected ? LED_CONN_BLINK_MS : LED_SCAN_BLINK_MS;
    if ((nowMs - ledToggleMs) >= interval)
    {
        ledToggleMs = nowMs;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}

// ============================================================================
// Slave state machine
// ============================================================================

/**
 * @brief Non-blocking slave state machine; must be called from loop().
 *
 * State flow:
 *   WIFI_INIT → CHANNEL_SCAN → CHANNEL_WAIT → ESPNOW_INIT → LISTENING
 *                                                                 ↑         |
 *                                                              RESCAN ←─────┘ (no master for RESCAN_INTERVAL_MS)
 *   ESPNOW_INIT → ERROR → WIFI_INIT (on esp_now_init failure, retries after delay)
 *
 * LISTENING handles all work: draining the receive queue, sending queued
 * responses, refreshing sensor data, and triggering a rescan when the master
 * has not been seen for RESCAN_INTERVAL_MS (covers the case where the slave
 * boots before the master and needs to rediscover the correct channel).
 */
static void slaveProcess(void)
{
    uint32_t nowMs = millis();

    switch (slaveState)
    {
    /* ------------------------------------------------------------------ */
    case SLAVE_STATE_WIFI_INIT:
        WiFi.mode(WIFI_STA);
        stateTimestampMs = nowMs;
        slaveState       = SLAVE_STATE_CHANNEL_SCAN;
        break;

    /* ------------------------------------------------------------------ */
    case SLAVE_STATE_CHANNEL_SCAN:
        if ((nowMs - stateTimestampMs) >= WIFI_RADIO_SETTLE_MS)
        {
            gprintf(gDBG, "[SLAVE] Scanning for master AP \"%s\"...\r\n",
                    MASTER_AP_SSID);
            WiFi.scanNetworks(/* async= */ true);
            stateTimestampMs = nowMs;
            slaveState       = SLAVE_STATE_CHANNEL_WAIT;
        }
        break;

    /* ------------------------------------------------------------------ */
    case SLAVE_STATE_CHANNEL_WAIT:
    {
        int16_t scanResult = (int16_t)WiFi.scanComplete();
        if (scanResult == (int16_t)WIFI_SCAN_RUNNING)
        {
            if ((nowMs - stateTimestampMs) >= CHANNEL_SCAN_TIMEOUT_MS)
            {
                gprintf(gDBG, "[SLAVE] Scan timed out – using ch%u\r\n",
                        (uint32_t)DEFAULT_ESPNOW_CHANNEL);
                WiFi.scanDelete();
                masterChannel = DEFAULT_ESPNOW_CHANNEL;
                esp_wifi_set_channel(masterChannel, WIFI_SECOND_CHAN_NONE);
                slaveState = SLAVE_STATE_ESPNOW_INIT;
            }
        }
        else
        {
            if (scanResult > 0)
            {
                uint8_t netCount = (uint8_t)scanResult;
                for (uint8_t i = 0U; i < netCount; i++)
                {
                    /* WiFi.SSID() returns String – .c_str() extracts raw ptr */
                    if (strcmp(WiFi.SSID(i).c_str(), MASTER_AP_SSID) == 0)
                    {
                        masterChannel = (uint8_t)WiFi.channel(i);
                        gprintf(gDBG, "[SLAVE] Master AP found on ch%u\r\n",
                                (uint32_t)masterChannel);
                        break;
                    }
                }
            }
            else
            {
                gprintf(gDBG, "[SLAVE] Master AP not found – using ch%u\r\n",
                        (uint32_t)DEFAULT_ESPNOW_CHANNEL);
                masterChannel = DEFAULT_ESPNOW_CHANNEL;
            }
            WiFi.scanDelete();
            esp_wifi_set_channel(masterChannel, WIFI_SECOND_CHAN_NONE);
            slaveState = SLAVE_STATE_ESPNOW_INIT;
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case SLAVE_STATE_ESPNOW_INIT:
        memset(&sensorData, 0, sizeof(sensorData));
        sensorUpdateMs   = nowMs;
        stateTimestampMs = nowMs;

        if (initEspNowSlave())
        {
            rescanTimerMs = nowMs;
            slaveState    = SLAVE_STATE_LISTENING;
        }
        else
        {
            slaveState = SLAVE_STATE_ERROR;
        }
        break;

    /* ------------------------------------------------------------------ */
    case SLAVE_STATE_LISTENING:
        updateSensorData();
        processRxQueue();
        sendPending();
        if (masterConnected)
        {
            rescanTimerMs = nowMs; /* keep timer from expiring while connected */
        }
        else if ((nowMs - rescanTimerMs) >= RESCAN_INTERVAL_MS)
        {
            gprintf(gDBG, "[SLAVE] No master contact for %us – re-scanning\r\n",
                    (uint32_t)(RESCAN_INTERVAL_MS / 1000U));
            slaveState = SLAVE_STATE_RESCAN;
        }
        break;

    /* ------------------------------------------------------------------ */
    case SLAVE_STATE_RESCAN:
        esp_now_deinit();
        masterConnected = false;
        rxHead          = 0U;
        rxTail          = 0U;
        txHead          = 0U;
        txTail          = 0U;
        slaveState      = SLAVE_STATE_WIFI_INIT;
        break;

    /* ------------------------------------------------------------------ */
    case SLAVE_STATE_ERROR:
        if ((nowMs - stateTimestampMs) >= INIT_RETRY_DELAY_MS)
        {
            gprintf(gDBG, "[SLAVE] Retrying...\r\n");
            slaveState = SLAVE_STATE_WIFI_INIT;
        }
        break;

    /* ------------------------------------------------------------------ */
    default:
        slaveState = SLAVE_STATE_WIFI_INIT;
        break;
    }
}

// ============================================================================
// Arduino entry points
// ============================================================================

/**
 * @brief One-time setup: initialises the debug UART and prints a boot banner.
 *
 * ESP-NOW initialisation is deferred into SLAVE_STATE_INIT so failures are
 * handled gracefully within the state machine.
 */
void setup(void)
{
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); /* disable brownout on weak PSU */
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    initUartBaud(gDBG, 115200U);
    esp_log_level_set("*", ESP_LOG_NONE);       /* suppress ESP-IDF radio logs */
    clear_screen(gDBG);
    gprintf(gDBG, "\r\n========================================\r\n");
    gprintf(gDBG, "  ESP-NOW Slave Node  v1.0\r\n");
    gprintf(gDBG, "  Home Monitor – sensor node\r\n");
    gprintf(gDBG, "========================================\r\n\r\n");
}

/**
 * @brief Main loop: drives the slave state machine on every iteration.
 */
void loop(void)
{
    updateLed();
    slaveProcess();
}

//=============================================================================
