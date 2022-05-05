#ifndef CAN_MOCK_HPP_
#define CAN_MOCK_HPP_

#include <stdint.h>

namespace can_mock
{
    /**
     * \brief CAN frame type (standard/extended)
     */
    typedef enum {
        CAN_frame_std = 0, /**< Standard frame, using 11 bit identifer. */
        CAN_frame_ext = 1  /**< Extended frame, using 29 bit identifer. */
    } CAN_frame_format_t;

    /**
     * \brief CAN RTR
     */
    typedef enum {
        CAN_no_RTR = 0, /**< No RTR frame. */
        CAN_RTR = 1     /**< RTR frame. */
    } CAN_RTR_t;

    /** \brief Frame information record type */
    typedef union {
        uint32_t U; /**< \brief Unsigned access */
        struct {
            uint8_t DLC : 4;               /**< \brief [3:0] DLC, Data length container */
            unsigned int unknown_2 : 2;    /**< \brief \internal unknown */
            CAN_RTR_t RTR : 1;             /**< \brief [6:6] RTR, Remote Transmission Request */
            CAN_frame_format_t FF : 1;     /**< \brief [7:7] Frame Format, see# CAN_frame_format_t*/
            unsigned int reserved_24 : 24; /**< \brief \internal Reserved */
        } B;
    } CAN_FIR_t;

    /** \brief CAN Frame structure */
    typedef struct {
        CAN_FIR_t FIR;  /**< \brief Frame information record*/
        uint32_t MsgID; /**< \brief Message ID */
        union {
            uint8_t u8[8];   /**< \brief Payload byte access*/
            uint32_t u32[2]; /**< \brief Payload u32 access*/
        } data;
    } CAN_frame_t;

    typedef enum {
        Dual_Mode=0, 							/**< \brief The dual acceptance filter option is enabled (two filters, each with the length of 16 bit are active) */
        Single_Mode=1 							/**< \brief The single acceptance filter option is enabled (one filter with the length of 32 bit is active) */
    } CAN_filter_mode_t;

    /** \brief CAN Filter structure */
    typedef struct {
        CAN_filter_mode_t 	FM:1;          		/**< \brief [0:0] Filter Mode */
        uint8_t 			ACR0;				/**< \brief Acceptance Code Register ACR0 */
        uint8_t 			ACR1;				/**< \brief Acceptance Code Register ACR1 */
        uint8_t 			ACR2;				/**< \brief Acceptance Code Register ACR2 */
        uint8_t 			ACR3;				/**< \brief Acceptance Code Register ACR3 */
        uint8_t 			AMR0;				/**< \brief Acceptance Mask Register AMR0 */
        uint8_t 			AMR1;				/**< \brief Acceptance Mask Register AMR1 */
        uint8_t 			AMR2;				/**< \brief Acceptance Mask Register AMR2 */
        uint8_t 			AMR3;				/**< \brief Acceptance Mask Register AMR3 */
    } CAN_filter_t;
}

#endif /* CAN_MOCK_HPP_ */