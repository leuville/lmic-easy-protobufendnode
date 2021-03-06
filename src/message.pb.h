/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.5 */

#ifndef PB_LEUVILLE_MESSAGE_PB_H_INCLUDED
#define PB_LEUVILLE_MESSAGE_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum _leuville_Type { 
    leuville_Type_PING = 0, 
    leuville_Type_BUTTON = 1 
} leuville_Type;

/* Struct definitions */
typedef struct _leuville_Downlink { 
    uint16_t pingDelay; /* seconds */
} leuville_Downlink;

typedef struct _leuville_Uplink { 
    leuville_Type type; 
    uint8_t battery; 
} leuville_Uplink;


/* Helper constants for enums */
#define _leuville_Type_MIN leuville_Type_PING
#define _leuville_Type_MAX leuville_Type_BUTTON
#define _leuville_Type_ARRAYSIZE ((leuville_Type)(leuville_Type_BUTTON+1))


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define leuville_Uplink_init_default             {_leuville_Type_MIN, 0}
#define leuville_Downlink_init_default           {0}
#define leuville_Uplink_init_zero                {_leuville_Type_MIN, 0}
#define leuville_Downlink_init_zero              {0}

/* Field tags (for use in manual encoding/decoding) */
#define leuville_Downlink_pingDelay_tag          1
#define leuville_Uplink_type_tag                 1
#define leuville_Uplink_battery_tag              2

/* Struct field encoding specification for nanopb */
#define leuville_Uplink_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UENUM,    type,              1) \
X(a, STATIC,   SINGULAR, UINT32,   battery,           2)
#define leuville_Uplink_CALLBACK NULL
#define leuville_Uplink_DEFAULT NULL

#define leuville_Downlink_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   pingDelay,         1)
#define leuville_Downlink_CALLBACK NULL
#define leuville_Downlink_DEFAULT NULL

extern const pb_msgdesc_t leuville_Uplink_msg;
extern const pb_msgdesc_t leuville_Downlink_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define leuville_Uplink_fields &leuville_Uplink_msg
#define leuville_Downlink_fields &leuville_Downlink_msg

/* Maximum encoded size of messages (where known) */
#define leuville_Downlink_size                   4
#define leuville_Uplink_size                     5

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
