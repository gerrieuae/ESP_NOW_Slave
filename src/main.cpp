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
#include <stdint.h>
#include <string.h>
#include "gprintf.h"

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
    SLAVE_STATE_INIT,      /**< Initialise WiFi and ESP-NOW.                  */
    SLAVE_STATE_LISTENING, /**< Operational: handle requests, update data.    */
    SLAVE_STATE_ERROR,     /**< Unrecoverable error – retry after delay.      */
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
static slave_state_t slaveState        = SLAVE_STATE_INIT;

/** @brief millis() recorded when the current state was entered. */
static uint32_t      stateTimestampMs  = 0U;

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

// ============================================================================
// Forward declarations
// ============================================================================

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
        return;

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
}

/**
 * @brief Drains the transmit queue by sending each pending response packet.
 */
static void sendPending(void)
{
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
 * @brief Initialises WiFi in station mode, starts ESP-NOW, and derives the
 *        slave's node ID from its MAC address.
 *
 * The node ID is set to the least-significant byte of the device MAC,
 * guaranteeing uniqueness across all ESP32 slaves without configuration.
 *
 * @return true on success, false if any ESP-NOW step failed.
 */
static bool initEspNowSlave(void)
{
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    uint8_t ownMac[MAC_ADDR_LEN];
    WiFi.macAddress(ownMac);
    slaveNodeId = ownMac[MAC_ADDR_LEN - 1U]; /* last byte = unique-enough ID */

    if (esp_now_init() != ESP_OK)
    {
        gprintf(gDBG, "[SLAVE] esp_now_init() failed\r\n");
        return false;
    }

    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    char macStr[18];
    macToString(ownMac, macStr);
    gprintf(gDBG, "[SLAVE] ESP-NOW ready. MAC=%s  NodeID=%u\r\n",
            macStr, (uint32_t)slaveNodeId);
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
// Slave state machine
// ============================================================================

/**
 * @brief Non-blocking slave state machine; must be called from loop().
 *
 * State flow:
 *   INIT → LISTENING (normal)
 *   INIT → ERROR → INIT (on esp_now_init failure, retries after delay)
 *
 * LISTENING handles all work: draining the receive queue, sending queued
 * responses, and refreshing sensor data on a timer.
 */
static void slaveProcess(void)
{
    uint32_t nowMs = millis();

    switch (slaveState)
    {
    /* ------------------------------------------------------------------ */
    case SLAVE_STATE_INIT:
        memset(&sensorData, 0, sizeof(sensorData));
        sensorUpdateMs   = nowMs;
        stateTimestampMs = nowMs;

        if (initEspNowSlave())
        {
            slaveState = SLAVE_STATE_LISTENING;
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
        break;

    /* ------------------------------------------------------------------ */
    case SLAVE_STATE_ERROR:
        if ((nowMs - stateTimestampMs) >= INIT_RETRY_DELAY_MS)
        {
            gprintf(gDBG, "[SLAVE] Retrying initialisation...\r\n");
            slaveState = SLAVE_STATE_INIT;
        }
        break;

    /* ------------------------------------------------------------------ */
    default:
        slaveState = SLAVE_STATE_INIT;
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
    initUartBaud(gDBG, 115200U);
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
    slaveProcess();
}

//=============================================================================
