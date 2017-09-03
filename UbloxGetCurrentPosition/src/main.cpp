
#include <iostream>
//#include <vector>
//#include <mutex>
#include <deque>

//#include "../../libraries/helper/helper.hpp"
#include "../../libraries/ublox/ublox.hpp"

struct FromGPS {
  int32_t longitude; // [10^-7 deg]
  int32_t latitude; // [10^-7 deg]
  float z; // height above sea level [m], downward positive
  float velocity[3]; // [m/s]
  uint8_t gps_status; // 3: pos & vel OK 2: only pos OK 1: only vel OK 0: unavailable
} __attribute__((packed));



int main(int argc, char const *argv[])
{

  int fd = open("/dev/gps_fifo", O_RDONLY | O_NONBLOCK);

  for(;;)
  {
    struct pollfd temp;
    temp.fd = fd;
    temp.events = POLLIN;
    poll(&temp, POLLIN, -1);
    unsigned char c;
    int r = (int) read(fd, &c, 1);
    if(r)
    {
        ProcessIncomingUBloxByte(c);
    }

    if(UBXNewDataAvailable())
    {
        static int i = 0; // total number of data received
        //static int n = 0; // number of data stored
        const int N = 100; // number of data to store
        static std::deque<struct UBXPayload> v_gps;
        i++;

        const struct UBXPayload * struct_ptr;
        struct_ptr = UBXPayload();
        //printf("\n lon:%u lat:%u height:%f v:[%f][%f][%f] stat:%u",
        //       struct_ptr->longitude, struct_ptr->latitude, struct_ptr->z,
        //       struct_ptr->velocity[0], struct_ptr->velocity[1], struct_ptr->velocity[2],
        //       struct_ptr->gps_status);

        struct UBXPayload temp_s;
        temp_s.longitude = struct_ptr->longitude;
        temp_s.latitude = struct_ptr->latitude;
        temp_s.z = struct_ptr -> z;
        temp_s.gps_status = struct_ptr->gps_status;
        for (int i=0;i<3;i++)
        {
            temp_s.velocity[i] = struct_ptr->velocity[i];
        }

        if(v_gps.size() >= N){
          v_gps.pop_front();
        }
        v_gps.push_back(temp_s);

        uint32_t avglon = 0, avglat = 0, avgz = 0;
        for(struct UBXPayload gps : v_gps){
          avglon += gps.longitude-uint32_t(gps.longitude/10000)*10000; // drop high digits
          avglat += gps.latitude-uint32_t(gps.latitude/10000)*10000; // drop high digits
          avgz += gps.z;
        }
        avglon = uint32_t(v_gps.back().longitude/10000)*10000 + avglon/v_gps.size();
        avglat = uint32_t(v_gps.back().latitude/10000)*10000 + avglat/v_gps.size();
        avgz = avgz/v_gps.size();

        std::cout << "i:" << i << " n:" << v_gps.size();
        std::cout << " avglon:" << avglon << " avglat:" << avglat << " avgz:" << avgz;
        std::cout << " lon:" << v_gps.back().longitude;
        std::cout << " lat:" << v_gps.back().latitude;
        std::cout << " z:" << v_gps.back().z << std::endl;

        ClearUBXNewDataFlags();
    }
  }
return 0;
}
