#pragma once

#include <cstdio>
#include <cstring>
#include <string>

#include "system/hardware/base.h"
#include "common/util.h"

#if QCOM2
#include "system/hardware/tici/hardware.h"
#define Hardware HardwareTici
#else
#include "system/hardware/pc/hardware.h"
#define Hardware HardwarePC
#endif

namespace Path {
  inline std::string openpilot_prefix() {
    return util::getenv("OPENPILOT_PREFIX", "");
  }

  inline std::string comma_home() {
    return util::getenv("HOME") + "/.comma" + Path::openpilot_prefix();
  }

  inline bool external_storage_available() {
    static constexpr char mount_point[] = "/mnt/external_realdata";
    bool mounted = false;
    if (FILE *fp = fopen("/proc/mounts", "re")) {
      char line[512] = {0};
      while (fgets(line, sizeof(line), fp) != nullptr) {
        char parsed_path[256] = {0};
        if (sscanf(line, "%*s %255s %*s %*s %*d %*d", parsed_path) == 1 && strcmp(parsed_path, mount_point) == 0) {
          mounted = true;
          break;
        }
      }
      fclose(fp);
    }
    return mounted && access(mount_point, W_OK) == 0;
  }

  inline std::string external_realdata() {
    static const std::string external_path = "/mnt/external_realdata";
    return external_storage_available() ? external_path : "/data/media/0/realdata";
  }


  inline std::string log_root() {
    if (const char *env = getenv("LOG_ROOT")) {
      return env;
    }
    return Hardware::PC() ? Path::comma_home() + "/media/0/realdata" : Path::external_realdata();
  }

  inline std::string params() {
    return util::getenv("PARAMS_ROOT", Hardware::PC() ? (Path::comma_home() + "/params") : "/data/params");
  }

  inline std::string rsa_file() {
    return Hardware::PC() ? Path::comma_home() + "/persist/comma/id_rsa" : "/persist/comma/id_rsa";
  }

  inline std::string swaglog_ipc() {
    return "ipc:///tmp/logmessage" + Path::openpilot_prefix();
  }

  inline std::string download_cache_root() {
    if (const char *env = getenv("COMMA_CACHE")) {
      return env;
    }
    return "/tmp/comma_download_cache" + Path::openpilot_prefix() + "/";
  }

 inline std::string shm_path() {
    #ifdef __APPLE__
     return"/tmp";
    #else
     return "/dev/shm";
    #endif
 }

  inline std::string model_root() {
    return Hardware::PC() ? Path::comma_home() + "/media/0/models" : "/data/media/0/models";
  }
}  // namespace Path
