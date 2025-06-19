#ifndef _INCLUDE_
#define _INCLUDE_

#define TOPIC_ID "topic_becks723"

#define PARAM_TRANSMIT_INTERVAL "transmit_interval"
#define PARAM_TRANSMIT_MSG      "transmit_msg"

typedef std_msgs::msg::String message;

#define FH_LO 0xA5   // 帧头低字节
#define FH_HI 0x5A   // 帧头高字节
#define FT_LO 0x0D   // 帧尾低字节
#define FT_HI 0x0A   // 帧尾高字节

#endif