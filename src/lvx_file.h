#include <string>
#include <iostream>
#include <sstream>
#include <cstdint>
#include <vector>
#include <fstream>

#define FRM_POINTS_COUNT 45216

#pragma pack(push, 1)
class PubHeader
{
public:
    PubHeader()
    {

    }
    ~PubHeader()
    {

    }

    char singnate[16];
    char ver[4];
    std::uint32_t magic_doce;
};

class PriHeader
{
public:
    PriHeader()
    {

    }
    ~PriHeader()
    {

    }

    std::uint32_t frame_duration;
    std::uint8_t device_count;
};

class DeviceInfo
{
public:
    DeviceInfo()
    {

    }
    ~DeviceInfo()
    {

    }

    std::uint8_t lidar_broadcast_code[16];
    std::uint8_t hub_brocast_code[16];
    std::uint32_t lidar_id;
    std::uint8_t lidar_type;
    std::uint8_t device_type;
    std::uint8_t extrinsic_enable;
    float roll;
    float pitch;
    float yaw;
    float x;
    float y;
    float z;
};

class FrameHeader
{
public:
    FrameHeader()
    {

    }
    ~FrameHeader()
    {

    }

    std::uint64_t curr_offset;
    std::uint64_t next_offset;
    std::uint64_t frame_idx;
};

class BasePackHeader
{
public:
    BasePackHeader()
    {

    }
    ~BasePackHeader()
    {

    }

    std::uint8_t version;
    std::uint32_t lidar_id;
    std::uint8_t lidar_type;
    std::uint8_t timestamp_type;
    std::uint8_t timestamp[8];
    std::uint16_t udp_counter;
    std::uint8_t data_type;
    std::uint32_t length;
    std::uint8_t frame_counter;
    std::uint8_t reserve[4];
};

class ExtendRowPoint
{
public:
    ExtendRowPoint()
    {

    }
    ~ExtendRowPoint()
    {

    }

    std::int32_t x;
    std::int32_t y;
    std::int32_t z;
    std::uint8_t reflectivity;
    std::uint8_t tag;
};

class ExtendHalfRowPoint
{
public:
    ExtendHalfRowPoint()
    {

    }
    ~ExtendHalfRowPoint()
    {

    }

    std::int16_t x;
    std::int16_t y;
    std::int16_t z;
    std::uint8_t reflectivity;
    std::uint8_t tag;
};

class BasePackDetail
{
public:
    BasePackDetail()
    {

    }
    ~BasePackDetail()
    {

    }

    BasePackHeader header;
    std::vector<ExtendRowPoint> raw_point;
};

class BaseHalfPackDetail
{
public:
    BaseHalfPackDetail()
    {

    }
    ~BaseHalfPackDetail()
    {

    }

    BasePackHeader header;
    std::vector<ExtendHalfRowPoint> raw_point;
};
#pragma pack(pop)
