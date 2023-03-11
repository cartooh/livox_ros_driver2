//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <functional>
#include <memory>
#include <thread>
#include <condition_variable>
#include <mutex>

#include "lds_lvx.h"
#include "lvx_file.h"
#include "comm/pub_handler.h"

#include <unistd.h>
#include <sys/param.h>

namespace livox_ros {

// std::condition_variable LdsLvx::cv_;
// std::mutex LdsLvx::mtx_;
std::atomic_bool LdsLvx::is_file_end_(false);

LdsLvx::LdsLvx(double publish_freq) : Lds(publish_freq, kSourceLvxFile), is_initialized_(false) {
}

LdsLvx::~LdsLvx() {
}

int LdsLvx::Init(const char *lvx_path) {
  if (is_initialized_) {
    printf("Livox file data source is already inited!\n");
    return false;
  }

#ifdef BUILDING_ROS2
  DisableLivoxSdkConsoleLogger();
#endif

  printf("Lds lvx init lvx_path:%s.\n", lvx_path);
  inf_lvx2_.open(lvx_path, std::ios_base::binary);
  if (!inf_lvx2_)
  {
    char path[MAXPATHLEN];
    char *p = getcwd (path, sizeof(path));
    printf("Open %s @ %s file fail!\n", lvx_path, p);
    return false;
  }
  
  inf_lvx2_.seekg( 0, std::ios_base::end );
  total_frame_ = inf_lvx2_.tellg();
  inf_lvx2_.seekg( 0, std::ios_base::beg );
  
  ResetLds(kSourceLvxFile);
  
  PubHeader pubheader;
  inf_lvx2_.read((char*)(&pubheader), sizeof(pubheader));
  
  PriHeader priheader;
  inf_lvx2_.read((char*)(&priheader), sizeof(priheader));
  
  uint32_t valid_lidar_count_ = (int)priheader.device_count;
  if (!valid_lidar_count_ || (valid_lidar_count_ >= kMaxSourceLidar)) {
    inf_lvx2_.close();
    printf("Lidar count error in %s : %d\n", lvx_path, valid_lidar_count_);
    return false;
  }
  printf("LvxFile[%s] have %d lidars\n", lvx_path, valid_lidar_count_);
  
  for (uint32_t i = 0; i < valid_lidar_count_; i++) {
    DeviceInfo devinfo;
    inf_lvx2_.read((char*)(&devinfo), sizeof(devinfo));
    uint32_t handle = devinfo.lidar_id;
    
    uint8_t index = 0;
    int8_t ret = cache_index_.GetFreeIndex(kLivoxLidarType, handle, index);
    if (ret != 0) {
      std::cout << "failed to get free index, lidar ip: " << IpNumToString(handle) << std::endl;
      continue;
    }
    LidarDevice& lidar = lidars_[index];
    lidar.lidar_type = kLivoxLidarType;
    lidar.connect_state = kConnectStateSampling;
    lidar.handle = handle;
    printf("lidar.lidar_type: %d; devinfo.lidar_type: %d, device_type:%d\n", lidar.lidar_type, devinfo.lidar_type, devinfo.device_type);
    

    LidarExtParameter lidar_param;
    lidar_param.handle = handle;
    lidar_param.lidar_type  = kLivoxLidarType;
    lidar_param.param.roll  = devinfo.roll;
    lidar_param.param.pitch = devinfo.pitch;
    lidar_param.param.yaw   = devinfo.yaw;
    lidar_param.param.x     = devinfo.x;
    lidar_param.param.y     = devinfo.y;
    lidar_param.param.z     = devinfo.z;
    pub_handler().AddLidarsExtParam(lidar_param);
  }

  t_read_lvx_ =
      std::make_shared<std::thread>(std::bind(&LdsLvx::ReadLvxFile, this));
  
  is_initialized_ = true;
  
  StartRead();
  
  return true;
}

void LdsLvx::OnPointCloudsFrameCallback(uint32_t frame_index, uint32_t total_frame, PointFrame *point_cloud_frame, void *client_data) {
  if (!point_cloud_frame) {
    printf("Point clouds frame call back failed, point cloud frame is nullptr.\n");
    return;
  }

  LdsLvx* lds = static_cast<LdsLvx*>(client_data);
  if (lds == nullptr) {
    printf("Point clouds frame call back failed, client data is nullptr.\n");
    return;
  }

  lds->StorageLvxPointData(point_cloud_frame);

  if (frame_index == total_frame) {
    is_file_end_.store(true);
  }
}

void LdsLvx::ReadLvxFile() {
  while (!start_read_lvx_);
  printf("Start to read lvx file.\n");
  
  uint8_t line_num = kLineNumberHAP;
  
  auto start_time = std::chrono::high_resolution_clock::now();
  int frame_cnt = 0;
  
  while (!inf_lvx2_.eof())
  {
    FrameHeader fraheader;
    inf_lvx2_.read((char*)(&fraheader), sizeof(fraheader));
    
    //         std::cout << "Frm curr offset:" << fraheader.curr_offset << std::endl;
    //         std::cout << "Next offset:" << fraheader.next_offset << std::endl;
    //         std::cout << "Frm idx:" << fraheader.frame_idx << std::endl;
    
    int bindx = 0;
    
    while (bindx < (int)(fraheader.next_offset - fraheader.curr_offset - sizeof(fraheader)))
    {
      BasePackHeader pheader;
      inf_lvx2_.read((char*)(&pheader), sizeof(pheader));
      bindx += sizeof(pheader);
      
      
      PointFrame point_cloud_frame;
      std::vector<PointXyzlt> points_clouds;
      
      point_cloud_frame.lidar_num = 1;
      PointPacket &pkt = point_cloud_frame.lidar_point[0];
      pkt.handle = pheader.lidar_id;
      pkt.lidar_type = LidarProtoType::kLivoxLidarType; 
      
      
      
      //             std::cout << "Data type:" << (int)pheader.data_type << std::endl;
      //             std::cout << "Length:" << pheader.length << std::endl;
      //             std::cout << "Frm counter:" << (int)pheader.frame_counter << std::endl;
      
      //long long int ts = *reinterpret_cast<long long int*>(&pheader.timestamp[0]);
      //point_cloud_frame.base_time = ts;
      point_cloud_frame.base_time = std::chrono::high_resolution_clock::now().time_since_epoch().count();
      
      // printf("timestampe: %lld\n", ts);
      
      
      if (pheader.data_type == 1)
      {
        int pcount = pheader.length / 14;
        points_clouds.resize(pcount);
        pkt.points_num = pcount;
        pkt.points = points_clouds.data();
        //printf("pcount: %d\n", pcount);
        
        for (int i = 0; i < pcount; i++)
        {
          ExtendRowPoint pdetail;
          inf_lvx2_.read((char*)(&pdetail), sizeof(pdetail));
          PointXyzlt &p = points_clouds[i];
          
          p.x = pdetail.x * 0.001f;
          p.y = pdetail.y * 0.001f;
          p.z = pdetail.z * 0.001f;
          p.intensity = pdetail.reflectivity;
          p.line = i % line_num;
          p.tag = pdetail.tag;
          //p.offset_time = ts + i;
          p.offset_time = std::chrono::high_resolution_clock::now().time_since_epoch().count();
          
          /*
          fdata.push_back(pdetail.x * 0.001f);
          fdata.push_back(pdetail.y * 0.001f);
          fdata.push_back(pdetail.z * 0.001f);
          fdata.push_back(pdetail.reflectivity);
          fdata.push_back(pdetail.tag);
          */
          
        }
      } else if (pheader.data_type == 2) {
        int pcount = pheader.length / 8;
        points_clouds.resize(pcount);
        //printf("pcount: %d\n", pcount);
        for (int i = 0; i < pcount; i++)
        {
          ExtendHalfRowPoint pdetail;
          inf_lvx2_.read((char*)(&pdetail), sizeof(pdetail));
          
          /*
          fdata.push_back(pdetail.x * 0.01f);
          fdata.push_back(pdetail.y * 0.01f);
          fdata.push_back(pdetail.z * 0.01f);
          fdata.push_back(pdetail.reflectivity);
          fdata.push_back(pdetail.tag);
          */
        }
      } else {
        std::cout << "Error:Can not surport this data type---" << (int)pheader.data_type << std::endl;
        break;
      }
      
      StorageLvxPointData(&point_cloud_frame);
      
      
      bindx += pheader.length;
      
    }
    
    ++frame_cnt;
    auto diff = std::chrono::milliseconds(50) * frame_cnt + start_time - std::chrono::high_resolution_clock::now();
    if(diff > std::chrono::milliseconds(0))
    {
      std::this_thread::sleep_for(diff);
    }
  }
  printf("done\n");
  inf_lvx2_.close();
}

}  // namespace livox_ros


