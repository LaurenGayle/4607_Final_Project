/*
 * BTstack configuration for Demo_Scripts
 * (Classic Bluetooth A2DP sink for audio + AVRCP for song titles).
 * BTstack requires this file to be on the include path.
 */
#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

/* ENABLE_CLASSIC is defined by the pico_btstack_classic CMake library */

// Logging
#define ENABLE_LOG_ERROR
#define ENABLE_PRINTF_HEXDUMP

// BTstack features
#define HAVE_EMBEDDED_TIME_MS
#define HAVE_ASSERT
#define HCI_RESET_RESEND_TIMEOUT_MS 1000

// HCI buffers (values used by the official Pico W examples)
#define HCI_OUTGOING_PRE_BUFFER_SIZE 4
#define HCI_ACL_PAYLOAD_SIZE (1691 + 4)
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT 4
#define MAX_NR_CONTROLLER_ACL_BUFFERS 3
#define MAX_NR_CONTROLLER_SCO_PACKETS 3
#define ENABLE_HCI_CONTROLLER_TO_HOST_FLOW_CONTROL
#define HCI_HOST_ACL_PACKET_LEN 1024
#define HCI_HOST_ACL_PACKET_NUM 3
#define HCI_HOST_SCO_PACKET_LEN 120
#define HCI_HOST_SCO_PACKET_NUM 3

// Static memory pools
#define MAX_NR_HCI_CONNECTIONS 2
#define MAX_NR_L2CAP_SERVICES 4            /* SDP, AVDTP, AVCTP (AVRCP) */
#define MAX_NR_L2CAP_CHANNELS 6            /* AVDTP signaling + media, AVRCP, SDP */
#define MAX_NR_RFCOMM_MULTIPLEXERS 0
#define MAX_NR_RFCOMM_SERVICES 0
#define MAX_NR_RFCOMM_CHANNELS 0
#define MAX_NR_BNEP_SERVICES 0
#define MAX_NR_BNEP_CHANNELS 0
#define MAX_NR_HFP_CONNECTIONS 0
#define MAX_NR_HID_HOST_CONNECTIONS 0
#define MAX_NR_AVDTP_CONNECTIONS 1
#define MAX_NR_AVDTP_STREAM_ENDPOINTS 1
#define MAX_NR_AVRCP_CONNECTIONS 2         /* one each for controller and target role */
#define MAX_NR_AVRCP_BROWSING_CONNECTIONS 0
#define MAX_NR_GOEP_SERVER_CONNECTIONS 0
#define MAX_NR_GOEP_SERVER_SERVICES 0
#define MAX_NR_SERVICE_RECORD_ITEMS 4      /* A2DP sink, AVRCP controller, AVRCP target */
#define MAX_NR_BTSTACK_LINK_KEY_DB_MEMORY_ENTRIES 2
#define MAX_NR_GATT_CLIENTS 0
#define MAX_NR_WHITELIST_ENTRIES 1
#define MAX_NR_SM_LOOKUP_ENTRIES 0

// Link keys are stored in flash so a paired laptop reconnects after reset
#define NVM_NUM_LINK_KEYS 16
#define NVM_NUM_DEVICE_DB_ENTRIES 16

#endif
