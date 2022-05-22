/*
 * Copyright (C) 2019-2021, The Linux Foundation. All rights reserved.
 * Not a Contribution.
 *
 * Copyright (C) 2021 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "android.hardware.usb@1.3-service-spes"

#include <android-base/logging.h>
#include <assert.h>
#include <chrono>
#include <dirent.h>
#include <pthread.h>
#include <regex>
#include <stdio.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>

#include <cutils/uevent.h>
#include <hidl/HidlTransportSupport.h>
#include <linux/usb/ch9.h>
#include <sys/epoll.h>
#include <utils/Errors.h>
#include <utils/StrongPointer.h>

#include "Usb.h"

namespace android {
namespace hardware {
namespace usb {
namespace V1_3 {
namespace implementation {

const char GOOGLE_USB_VENDOR_ID_STR[] = "18d1";
const char GOOGLE_USBC_35_ADAPTER_UNPLUGGED_ID_STR[] = "5029";

Return<bool> Usb::enableUsbDataSignal(bool enable) {
  bool result = true;

  ALOGI("Userspace turn %s USB data signaling", enable ? "on" : "off");

  if (enable) {
    if (!WriteStringToFile("1", mDevicePath + USB_DATA_PATH)) {
      ALOGE("Not able to turn on usb connection notification");
      result = false;
    }

    if (!WriteStringToFile(mGadgetName, PULLUP_PATH)) {
      ALOGW("Gadget cannot be pulled up");
    }
  } else {
    if (!WriteStringToFile("1", mDevicePath + ID_PATH)) {
      ALOGW("Not able to turn off host mode");
    }

    if (!WriteStringToFile("0", mDevicePath + VBUS_PATH)) {
      ALOGW("Not able to set Vbus state");
    }

    if (!WriteStringToFile("0", mDevicePath + USB_DATA_PATH)) {
      ALOGE("Not able to turn off usb connection notification");
      result = false;
    }

    if (!WriteStringToFile("none", PULLUP_PATH)) {
      ALOGW("Gadget cannot be pulled down");
    }
  }

  return result;
}

// Set by the signal handler to destroy the thread
volatile bool destroyThread;

static void checkUsbWakeupSupport(struct Usb *usb);
static void checkUsbDeviceAutoSuspend(const std::string& devicePath);
static bool checkUsbInterfaceAutoSuspend(const std::string& devicePath,
        const std::string &intf);

static int32_t readFile(const std::string &filename, std::string *contents) {
  FILE *fp;
  ssize_t read = 0;
  char *line = NULL;
  size_t len = 0;

  fp = fopen(filename.c_str(), "r");
  if (fp != NULL) {
    if ((read = getline(&line, &len, fp)) != -1) {
      char *pos;
      if ((pos = strchr(line, '\n')) != NULL) *pos = '\0';
      *contents = line;
    }
    free(line);
    fclose(fp);
    return 0;
  } else {
    ALOGE("fopen failed in readFile %s, errno=%d", filename.c_str(), errno);
  }

  return -1;
}

static int32_t writeFile(const std::string &filename,
                         const std::string &contents) {
  FILE *fp;
  int ret;

  fp = fopen(filename.c_str(), "w");
  if (fp != NULL) {
    ret = fputs(contents.c_str(), fp);
    fclose(fp);
    if (ret == EOF) {
      ALOGE("fputs failed in writeFile %s", filename.c_str());
      return -1;
    }
    return 0;
  } else {
    ALOGE("fopen failed in writeFile %s, errno=%d", filename.c_str(), errno);
  }

  return -1;
}

std::string appendRoleNodeHelper(const std::string &portName,
                                 PortRoleType type) {
  std::string node("/sys/class/typec/" + portName);

  switch (type) {
    case PortRoleType::DATA_ROLE:
      return node + "/data_role";
    case PortRoleType::POWER_ROLE:
      return node + "/power_role";
    case PortRoleType::MODE:
      return node + "/port_type";
    default:
      return "";
  }
}

std::string convertRoletoString(PortRole role) {
  if (role.type == PortRoleType::POWER_ROLE) {
    if (role.role == static_cast<uint32_t>(PortPowerRole::SOURCE))
      return "source";
    else if (role.role == static_cast<uint32_t>(PortPowerRole::SINK))
      return "sink";
  } else if (role.type == PortRoleType::DATA_ROLE) {
    if (role.role == static_cast<uint32_t>(PortDataRole::HOST)) return "host";
    if (role.role == static_cast<uint32_t>(PortDataRole::DEVICE))
      return "device";
  } else if (role.type == PortRoleType::MODE) {
    if (role.role == static_cast<uint32_t>(PortMode_1_1::UFP)) return "sink";
    if (role.role == static_cast<uint32_t>(PortMode_1_1::DFP)) return "source";
  }
  return "none";
}

void extractRole(std::string *roleName) {
  std::size_t first, last;

  first = roleName->find("[");
  last = roleName->find("]");

  if (first != std::string::npos && last != std::string::npos) {
    *roleName = roleName->substr(first + 1, last - first - 1);
  }
}

void switchToDrp(const std::string &portName) {
  std::string filename =
      appendRoleNodeHelper(std::string(portName.c_str()), PortRoleType::MODE);
  FILE *fp;

  if (filename != "") {
    fp = fopen(filename.c_str(), "w");
    if (fp != NULL) {
      int ret = fputs("dual", fp);
      fclose(fp);
      if (ret == EOF)
        ALOGE("Fatal: Error while switching back to drp");
    } else {
      ALOGE("Fatal: Cannot open file to switch back to drp");
    }
  } else {
    ALOGE("Fatal: invalid node type");
  }
}

bool switchMode(const hidl_string &portName,
                             const PortRole &newRole, struct Usb *usb) {
  std::string filename =
       appendRoleNodeHelper(std::string(portName.c_str()), newRole.type);
  std::string written;
  FILE *fp;
  bool roleSwitch = false;

  if (filename == "") {
    ALOGE("Fatal: invalid node type");
    return false;
  }

  fp = fopen(filename.c_str(), "w");
  if (fp != NULL) {
    // Hold the lock here to prevent loosing connected signals
    // as once the file is written the partner added signal
    // can arrive anytime.
    pthread_mutex_lock(&usb->mPartnerLock);
    usb->mPartnerUp = false;
    int ret = fputs(convertRoletoString(newRole).c_str(), fp);
    fclose(fp);

    if (ret != EOF) {
      struct timespec   to;
      struct timespec   now;

wait_again:
      clock_gettime(CLOCK_MONOTONIC, &now);
      to.tv_sec = now.tv_sec + PORT_TYPE_TIMEOUT;
      to.tv_nsec = now.tv_nsec;

      int err = pthread_cond_timedwait(&usb->mPartnerCV, &usb->mPartnerLock, &to);
      // There are no uevent signals which implies role swap timed out.
      if (err == ETIMEDOUT) {
        ALOGI("uevents wait timedout");
      // Sanity check.
      } else if (!usb->mPartnerUp) {
        goto wait_again;
      // Role switch succeeded since usb->mPartnerUp is true.
      } else {
        roleSwitch = true;
      }
    } else {
      ALOGI("Role switch failed while wrting to file");
    }
    pthread_mutex_unlock(&usb->mPartnerLock);
  }

  if (!roleSwitch)
    switchToDrp(std::string(portName.c_str()));

  return roleSwitch;
}

Usb::Usb(std::string deviceName, std::string gadgetName)
        : mLock(PTHREAD_MUTEX_INITIALIZER),
          mRoleSwitchLock(PTHREAD_MUTEX_INITIALIZER),
          mPartnerLock(PTHREAD_MUTEX_INITIALIZER),
          mPartnerUp(false),
          mContaminantPresence(false),
          mDevicePath(SOC_PATH + deviceName + "/"),
          mGadgetName(gadgetName) {
    pthread_condattr_t attr;
    if (pthread_condattr_init(&attr)) {
        ALOGE("pthread_condattr_init failed: %s", strerror(errno));
        abort();
    }
    if (pthread_condattr_setclock(&attr, CLOCK_MONOTONIC)) {
        ALOGE("pthread_condattr_setclock failed: %s", strerror(errno));
        abort();
    }
    if (pthread_cond_init(&mPartnerCV, &attr))  {
        ALOGE("pthread_cond_init failed: %s", strerror(errno));
        abort();
    }
    if (pthread_condattr_destroy(&attr)) {
        ALOGE("pthread_condattr_destroy failed: %s", strerror(errno));
        abort();
    }

}


Return<void> Usb::switchRole(const hidl_string &portName,
                             const V1_0::PortRole &newRole) {
  std::string filename =
      appendRoleNodeHelper(std::string(portName.c_str()), newRole.type);
  std::string written;
  FILE *fp;
  bool roleSwitch = false;

  if (filename == "") {
    ALOGE("Fatal: invalid node type");
    return Void();
  }

  pthread_mutex_lock(&mRoleSwitchLock);

  ALOGI("filename write: %s role:%s", filename.c_str(),
        convertRoletoString(newRole).c_str());

  if (newRole.type == PortRoleType::MODE) {
      roleSwitch = switchMode(portName, newRole, this);
  } else {
    fp = fopen(filename.c_str(), "w");
    if (fp != NULL) {
      int ret = fputs(convertRoletoString(newRole).c_str(), fp);
      fclose(fp);
      if ((ret != EOF) && !readFile(filename, &written)) {
        extractRole(&written);
        ALOGI("written: %s", written.c_str());
        if (written == convertRoletoString(newRole)) {
          roleSwitch = true;
        } else {
          ALOGE("Role switch failed");
        }
      } else {
        ALOGE("failed to update the new role");
      }
    } else {
      ALOGE("fopen failed");
    }
  }

  pthread_mutex_lock(&mLock);
  if (mCallback_1_0 != NULL) {
    Return<void> ret =
        mCallback_1_0->notifyRoleSwitchStatus(portName, newRole,
        roleSwitch ? Status::SUCCESS : Status::ERROR);
    if (!ret.isOk())
      ALOGE("RoleSwitchStatus error %s", ret.description().c_str());
  } else {
    ALOGE("Not notifying the userspace. Callback is not set");
  }
  pthread_mutex_unlock(&mLock);
  pthread_mutex_unlock(&mRoleSwitchLock);

  return Void();
}

Status getAccessoryConnected(const std::string &portName, std::string *accessory) {
  std::string filename =
    "/sys/class/typec/" + portName + "-partner/accessory_mode";

  if (readFile(filename, accessory)) {
    ALOGE("getAccessoryConnected: Failed to open filesystem node: %s",
          filename.c_str());
    return Status::ERROR;
  }

  return Status::SUCCESS;
}

Status getCurrentRoleHelper(const std::string &portName, bool connected,
                            PortRoleType type, uint32_t *currentRole) {
  std::string filename;
  std::string roleName;
  std::string accessory;

  // Mode

  if (type == PortRoleType::POWER_ROLE) {
    filename = "/sys/class/typec/" + portName + "/power_role";
    *currentRole = static_cast<uint32_t>(PortPowerRole::NONE);
  } else if (type == PortRoleType::DATA_ROLE) {
    filename = "/sys/class/typec/" + portName + "/data_role";
    *currentRole = static_cast<uint32_t>(PortDataRole::NONE);
  } else if (type == PortRoleType::MODE) {
    filename = "/sys/class/typec/" + portName + "/data_role";
    *currentRole = static_cast<uint32_t>(PortMode_1_1::NONE);
  } else {
    return Status::ERROR;
  }

  if (!connected) return Status::SUCCESS;

  if (type == PortRoleType::MODE) {
    if (getAccessoryConnected(portName, &accessory) != Status::SUCCESS) {
      return Status::ERROR;
    }
    if (accessory == "analog_audio") {
      *currentRole = static_cast<uint32_t>(PortMode_1_1::AUDIO_ACCESSORY);
      return Status::SUCCESS;
    } else if (accessory == "debug") {
      *currentRole = static_cast<uint32_t>(PortMode_1_1::DEBUG_ACCESSORY);
      return Status::SUCCESS;
    }
  }

  if (readFile(filename, &roleName)) {
    ALOGE("getCurrentRole: Failed to open filesystem node: %s",
          filename.c_str());
    return Status::ERROR;
  }

  extractRole(&roleName);

  if (roleName == "source") {
    *currentRole = static_cast<uint32_t>(PortPowerRole::SOURCE);
  } else if (roleName == "sink") {
    *currentRole = static_cast<uint32_t>(PortPowerRole::SINK);
  } else if (roleName == "host") {
    if (type == PortRoleType::DATA_ROLE)
      *currentRole = static_cast<uint32_t>(PortDataRole::HOST);
    else
      *currentRole = static_cast<uint32_t>(PortMode_1_1::DFP);
  } else if (roleName == "device") {
    if (type == PortRoleType::DATA_ROLE)
      *currentRole = static_cast<uint32_t>(PortDataRole::DEVICE);
    else
      *currentRole = static_cast<uint32_t>(PortMode_1_1::UFP);
  } else if (roleName != "none") {
    /* case for none has already been addressed.
     * so we check if the role isnt none.
     */
    return Status::UNRECOGNIZED_ROLE;
  }

  return Status::SUCCESS;
}

Status getTypeCPortNamesHelper(std::unordered_map<std::string, bool> *names) {
  DIR *dp;

  dp = opendir("/sys/class/typec");
  if (dp != NULL) {
    struct dirent *ep;

    while ((ep = readdir(dp))) {
      if (ep->d_type == DT_LNK) {
        if (std::string::npos == std::string(ep->d_name).find("-partner")) {
          std::unordered_map<std::string, bool>::const_iterator portName =
              names->find(ep->d_name);
          if (portName == names->end()) {
            names->insert({ep->d_name, false});
          }
        } else {
          (*names)[std::strtok(ep->d_name, "-")] = true;
        }
      }
    }
    closedir(dp);
    return Status::SUCCESS;
  }

  ALOGE("Failed to open /sys/class/typec");
  return Status::ERROR;
}

bool canSwitchRoleHelper(const std::string &portName, PortRoleType /*type*/) {
  std::string filename =
      "/sys/class/typec/" + portName + "-partner/supports_usb_power_delivery";
  std::string supportsPD;

  if (!readFile(filename, &supportsPD)) {
    if (supportsPD == "yes") {
      return true;
    }
  }

  return false;
}

/*
 * The caller of this method would reconstruct the V1_0::PortStatus
 * object if required.
 */
Status getPortStatusHelper(hidl_vec<PortStatus> *currentPortStatus_1_2,
    bool V1_0, struct Usb *usb) {
  std::unordered_map<std::string, bool> names;
  Status result = getTypeCPortNamesHelper(&names);
  int i = -1;

  if (result == Status::SUCCESS) {
    if (names.size() == 0) {
      ALOGI("Hardcode parameters for non-typec targets");
      currentPortStatus_1_2->resize(1);
      /*
       * Below assignments are done in accordance with the checks in VtsHalUsbV1_2TargetTest
       * so as to make the VTS testing pass for non typec targets.
       */
      (*currentPortStatus_1_2)[0].status_1_1.status.supportedModes = V1_0::PortMode::NONE;
      (*currentPortStatus_1_2)[0].status_1_1.status.currentMode = V1_0::PortMode::NONE;
    } else {
      currentPortStatus_1_2->resize(names.size());
    }
    for (std::pair<std::string, bool> port : names) {
      i++;
      ALOGI("%s", port.first.c_str());
      (*currentPortStatus_1_2)[i].status_1_1.status.portName = port.first;

      uint32_t currentRole;
      if (getCurrentRoleHelper(port.first, port.second,
                               PortRoleType::POWER_ROLE,
                               &currentRole) == Status::SUCCESS) {
        (*currentPortStatus_1_2)[i].status_1_1.status.currentPowerRole =
            static_cast<PortPowerRole>(currentRole);
      } else {
        ALOGE("Error while retreiving portNames");
        goto done;
      }

      if (getCurrentRoleHelper(port.first, port.second, PortRoleType::DATA_ROLE,
                               &currentRole) == Status::SUCCESS) {
        (*currentPortStatus_1_2)[i].status_1_1.status.currentDataRole =
            static_cast<PortDataRole>(currentRole);
      } else {
        ALOGE("Error while retreiving current port role");
        goto done;
      }

      if (getCurrentRoleHelper(port.first, port.second, PortRoleType::MODE,
                               &currentRole) == Status::SUCCESS) {
        (*currentPortStatus_1_2)[i].status_1_1.currentMode =
            static_cast<PortMode_1_1>(currentRole);
        (*currentPortStatus_1_2)[i].status_1_1.status.currentMode =
            static_cast<V1_0::PortMode>(currentRole);
      } else {
        ALOGE("Error while retreiving current data role");
        goto done;
      }

      (*currentPortStatus_1_2)[i].status_1_1.status.canChangeMode = true;
      (*currentPortStatus_1_2)[i].status_1_1.status.canChangeDataRole =
          port.second ? canSwitchRoleHelper(port.first, PortRoleType::DATA_ROLE)
                      : false;
      (*currentPortStatus_1_2)[i].status_1_1.status.canChangePowerRole =
          port.second
              ? canSwitchRoleHelper(port.first, PortRoleType::POWER_ROLE)
              : false;

      ALOGI("connected:%d canChangeMode:%d canChagedata:%d canChangePower:%d",
            port.second, (*currentPortStatus_1_2)[i].status_1_1.status.canChangeMode,
            (*currentPortStatus_1_2)[i].status_1_1.status.canChangeDataRole,
            (*currentPortStatus_1_2)[i].status_1_1.status.canChangePowerRole);

      if (V1_0) {
        (*currentPortStatus_1_2)[i].status_1_1.status.supportedModes = V1_0::PortMode::DFP;
      } else {
        (*currentPortStatus_1_2)[i].status_1_1.supportedModes =
	    PortMode_1_1::DRP | PortMode_1_1::AUDIO_ACCESSORY;
        (*currentPortStatus_1_2)[i].status_1_1.status.supportedModes = V1_0::PortMode::NONE;
        (*currentPortStatus_1_2)[i].status_1_1.status.currentMode = V1_0::PortMode::NONE;

        (*currentPortStatus_1_2)[i].supportedContaminantProtectionModes =
            ContaminantProtectionMode::FORCE_SINK | ContaminantProtectionMode::FORCE_DISABLE;
        (*currentPortStatus_1_2)[i].supportsEnableContaminantPresenceProtection =
            false;
        (*currentPortStatus_1_2)[i].supportsEnableContaminantPresenceDetection =
            false;
        (*currentPortStatus_1_2)[i].contaminantProtectionStatus =
            ContaminantProtectionStatus::FORCE_SINK;

        if (port.first != "port0") // moisture detection only on first port
          continue;

        std::string contaminantPresence;

        if (!usb->mContaminantStatusPath.empty() &&
                                !readFile(usb->mContaminantStatusPath, &contaminantPresence)) {
          if (contaminantPresence == "1") {
            (*currentPortStatus_1_2)[i].contaminantDetectionStatus =
                ContaminantDetectionStatus::DETECTED;
            ALOGI("moisture: Contaminant presence detected");
          }
          else {
            (*currentPortStatus_1_2)[i].contaminantDetectionStatus =
                ContaminantDetectionStatus::NOT_DETECTED;
          }
        } else {
          (*currentPortStatus_1_2)[i].supportedContaminantProtectionModes =
              ContaminantProtectionMode::NONE | ContaminantProtectionMode::NONE;
          (*currentPortStatus_1_2)[i].contaminantProtectionStatus =
              ContaminantProtectionStatus::NONE;
        }
      }
    }
    return Status::SUCCESS;
  }
done:
  return Status::ERROR;
}

Return<void> Usb::queryPortStatus() {
  hidl_vec<PortStatus> currentPortStatus_1_2;
  hidl_vec<V1_1::PortStatus_1_1> currentPortStatus_1_1;
  hidl_vec<V1_0::PortStatus> currentPortStatus;
  Status status;
  sp<IUsbCallback> callback_V1_2 = IUsbCallback::castFrom(mCallback_1_0);
  sp<V1_1::IUsbCallback> callback_V1_1 = V1_1::IUsbCallback::castFrom(mCallback_1_0);

  pthread_mutex_lock(&mLock);
  if (mCallback_1_0 != NULL) {
    if (callback_V1_1 != NULL) { // 1.1 or 1.2
      if (callback_V1_2 == NULL) { // 1.1 only
        status = getPortStatusHelper(&currentPortStatus_1_2, false, this);
        currentPortStatus_1_1.resize(currentPortStatus_1_2.size());
        for (unsigned long i = 0; i < currentPortStatus_1_2.size(); i++)
          currentPortStatus_1_1[i].status = currentPortStatus_1_2[i].status_1_1.status;
      }
      else  //1.2 only
        status = getPortStatusHelper(&currentPortStatus_1_2, false, this);
    } else { // 1.0 only
      status = getPortStatusHelper(&currentPortStatus_1_2, true, this);
      currentPortStatus.resize(currentPortStatus_1_2.size());
      for (unsigned long i = 0; i < currentPortStatus_1_2.size(); i++)
        currentPortStatus[i] = currentPortStatus_1_2[i].status_1_1.status;
    }

    Return<void> ret;

    if (callback_V1_2 != NULL)
      ret = callback_V1_2->notifyPortStatusChange_1_2(currentPortStatus_1_2, status);
    else if (callback_V1_1 != NULL)
      ret = callback_V1_1->notifyPortStatusChange_1_1(currentPortStatus_1_1, status);
    else
      ret = mCallback_1_0->notifyPortStatusChange(currentPortStatus, status);

    if (!ret.isOk())
      ALOGE("queryPortStatus_1_1 error %s", ret.description().c_str());
  } else {
    ALOGI("Notifying userspace skipped. Callback is NULL");
  }
  pthread_mutex_unlock(&mLock);
  return Void();
}

struct data {
  int uevent_fd;
  android::hardware::usb::V1_3::implementation::Usb *usb;
};

Return<void> callbackNotifyPortStatusChangeHelper(struct Usb *usb) {
  hidl_vec<PortStatus> currentPortStatus_1_2;
  Status status;
  Return<void> ret;
  sp<IUsbCallback> callback_V1_2 = IUsbCallback::castFrom(usb->mCallback_1_0);

  pthread_mutex_lock(&usb->mLock);
  status = getPortStatusHelper(&currentPortStatus_1_2, false, usb);
  ret = callback_V1_2->notifyPortStatusChange_1_2(currentPortStatus_1_2, status);

  if (!ret.isOk())
    ALOGE("notifyPortStatusChange_1_2 error %s", ret.description().c_str());

  pthread_mutex_unlock(&usb->mLock);
  return Void();
}

Return<void> Usb::enableContaminantPresenceDetection(const hidl_string &portName,
                                                     bool enable) {
  Return<void> ret;

  ret = callbackNotifyPortStatusChangeHelper(this);
  ALOGI("Contaminant Presence Detection should always be in enable mode");

  return Void();
}

Return<void> Usb::enableContaminantPresenceProtection(const hidl_string &portName,
                                                      bool enable) {
  Return<void> ret;

  ret = callbackNotifyPortStatusChangeHelper(this);
  ALOGI("Contaminant Presence Protection should always be in enable mode");

  return Void();
}

static void handle_typec_uevent(Usb *usb, const char *msg)
{
  ALOGI("uevent received %s", msg);

  // if (std::regex_match(cp, std::regex("(add)(.*)(-partner)")))
  if (!strncmp(msg, "add@", 4) && !strncmp(msg + strlen(msg) - 8, "-partner", 8)) {
     ALOGI("partner added");
     pthread_mutex_lock(&usb->mPartnerLock);
     usb->mPartnerUp = true;
     pthread_cond_signal(&usb->mPartnerCV);
     pthread_mutex_unlock(&usb->mPartnerLock);
  }

  std::string power_operation_mode;
  if (!readFile("/sys/class/typec/port0/power_operation_mode", &power_operation_mode)) {
    if (usb->mPowerOpMode == power_operation_mode) {
      ALOGI("uevent recieved for same device %s", power_operation_mode.c_str());
    } else if(power_operation_mode == "usb_power_delivery") {
      readFile("/config/usb_gadget/g1/configs/b.1/MaxPower", &usb->mMaxPower);
      readFile("/config/usb_gadget/g1/configs/b.1/bmAttributes", &usb->mAttributes);
      writeFile("/config/usb_gadget/g1/configs/b.1/MaxPower", "0");
      writeFile("/config/usb_gadget/g1/configs/b.1/bmAttributes", "0xc0");
    } else {
      if(!usb->mMaxPower.empty()) {
        writeFile("/config/usb_gadget/g1/configs/b.1/MaxPower", usb->mMaxPower.c_str());
        writeFile("/config/usb_gadget/g1/configs/b.1/bmAttributes", usb->mAttributes.c_str());
        usb->mMaxPower = "";
      }
    }

    usb->mPowerOpMode = power_operation_mode;
  }

  usb->queryPortStatus();
}

// process POWER_SUPPLY uevent for contaminant presence
static void handle_psy_uevent(Usb *usb, const char *msg)
{
  sp<IUsbCallback> callback_V1_2 = IUsbCallback::castFrom(usb->mCallback_1_0);
  hidl_vec<PortStatus> currentPortStatus_1_2;
  Status status;
  Return<void> ret;
  bool moisture_detected;
  std::string contaminantPresence;

  // don't bother parsing any further if caller doesn't support USB HAL 1.2
  // to report contaminant presence events
  if (callback_V1_2 == NULL)
    return;

  while (*msg) {
    if (!strncmp(msg, "POWER_SUPPLY_NAME=", 18)) {
      msg += 18;
      if (strcmp(msg, "usb")) // make sure we're looking at the correct uevent
        return;
      else
        break;
    }

    // advance to after the next \0
    while (*msg++) ;
  }

  // read moisture detection status from sysfs
  if (usb->mContaminantStatusPath.empty() ||
        readFile(usb->mContaminantStatusPath, &contaminantPresence))
    return;

  moisture_detected = (contaminantPresence[0] == '1');

  if (usb->mContaminantPresence != moisture_detected) {
    usb->mContaminantPresence = moisture_detected;

    status = getPortStatusHelper(&currentPortStatus_1_2, false, usb);
    ret = callback_V1_2->notifyPortStatusChange_1_2(currentPortStatus_1_2, status);
    if (!ret.isOk()) ALOGE("error %s", ret.description().c_str());
  }

  //Role switch is not in progress and port is in disconnected state
  if (!pthread_mutex_trylock(&usb->mRoleSwitchLock)) {
    for (unsigned long i = 0; i < currentPortStatus_1_2.size(); i++) {
      DIR *dp = opendir(std::string("/sys/class/typec/"
          + std::string(currentPortStatus_1_2[i].status_1_1.status.portName.c_str())
          + "-partner").c_str());
      if (dp == NULL) {
        //PortRole role = {.role = static_cast<uint32_t>(PortMode::UFP)};
        switchToDrp(currentPortStatus_1_2[i].status_1_1.status.portName);
      } else {
        closedir(dp);
      }
    }
    pthread_mutex_unlock(&usb->mRoleSwitchLock);
  }
}

static void uevent_event(uint32_t /*epevents*/, struct data *payload) {
  char msg[UEVENT_MSG_LEN + 2];
  int n;
  static std::regex add_regex("add@(/devices/platform/soc/.*dwc3/xhci-hcd\\.\\d\\.auto/"
                              "usb\\d/\\d-\\d(?:/[\\d\\.-]+)*)");
  static std::regex bind_regex("bind@(/devices/platform/soc/.*dwc3/xhci-hcd\\.\\d\\.auto/"
                               "usb\\d/\\d-\\d(?:/[\\d\\.-]+)*)/([^/]*:[^/]*)");

  n = uevent_kernel_multicast_recv(payload->uevent_fd, msg, UEVENT_MSG_LEN);
  if (n <= 0) return;
  if (n >= UEVENT_MSG_LEN) /* overflow -- discard */
    return;

  msg[n] = '\0';
  msg[n + 1] = '\0';

  std::cmatch match;

  if (strstr(msg, "typec/port")) {
    handle_typec_uevent(payload->usb, msg);
  } else if (strstr(msg, "power_supply/usb")) {
    handle_psy_uevent(payload->usb, msg + strlen(msg) + 1);
  } else if (std::regex_match(msg, match, add_regex)) {
    if (match.size() == 2) {
      std::csub_match submatch = match[1];
      checkUsbDeviceAutoSuspend("/sys" +  submatch.str());
    }
  } else if (!payload->usb->mIgnoreWakeup && std::regex_match(msg, match, bind_regex)) {
    if (match.size() == 3) {
      std::csub_match devpath = match[1];
      std::csub_match intfpath = match[2];
      checkUsbInterfaceAutoSuspend("/sys" + devpath.str(), intfpath.str());
    }
  }
}

void *work(void *param) {
  int epoll_fd, uevent_fd;
  struct epoll_event ev;
  int nevents = 0;
  struct data payload;

  ALOGE("creating thread");

  uevent_fd = uevent_open_socket(64 * 1024, true);

  if (uevent_fd < 0) {
    ALOGE("uevent_init: uevent_open_socket failed\n");
    return NULL;
  }

  payload.uevent_fd = uevent_fd;
  payload.usb = (android::hardware::usb::V1_3::implementation::Usb *)param;

  fcntl(uevent_fd, F_SETFL, O_NONBLOCK);

  ev.events = EPOLLIN;
  ev.data.ptr = (void *)uevent_event;

  epoll_fd = epoll_create(64);
  if (epoll_fd == -1) {
    ALOGE("epoll_create failed; errno=%d", errno);
    goto error;
  }

  if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, uevent_fd, &ev) == -1) {
    ALOGE("epoll_ctl failed; errno=%d", errno);
    goto error;
  }

  while (!destroyThread) {
    struct epoll_event events[64];

    nevents = epoll_wait(epoll_fd, events, 64, -1);
    if (nevents == -1) {
      if (errno == EINTR) continue;
      ALOGE("usb epoll_wait failed; errno=%d", errno);
      break;
    }

    for (int n = 0; n < nevents; ++n) {
      if (events[n].data.ptr)
        (*(void (*)(uint32_t, struct data *payload))events[n].data.ptr)(
            events[n].events, &payload);
    }
  }

  ALOGI("exiting worker thread");
error:
  close(uevent_fd);

  if (epoll_fd >= 0) close(epoll_fd);

  return NULL;
}

void sighandler(int sig) {
  if (sig == SIGUSR1) {
    destroyThread = true;
    ALOGI("destroy set");
    return;
  }
  signal(SIGUSR1, sighandler);
}

Return<void> Usb::setCallback(const sp<V1_0::IUsbCallback> &callback) {

  sp<V1_1::IUsbCallback> callback_V1_1 = V1_1::IUsbCallback::castFrom(callback);
  sp<IUsbCallback> callback_V1_2 = IUsbCallback::castFrom(callback);

  if (callback != NULL)
      if (callback_V1_1 == NULL)
          ALOGI("Registering 1.0 callback");

  pthread_mutex_lock(&mLock);
  /*
   * When both the old callback and new callback values are NULL,
   * there is no need to spin off the worker thread.
   * When both the values are not NULL, we would already have a
   * worker thread running, so updating the callback object would
   * be suffice.
   */
  if ((mCallback_1_0 == NULL && callback == NULL) ||
      (mCallback_1_0 != NULL && callback != NULL)) {
    /*
     * Always store as V1_0 callback object. Type cast to V1_1
     * when the callback is actually invoked.
     */
    mCallback_1_0 = callback;
    pthread_mutex_unlock(&mLock);
    return Void();
  }

  mCallback_1_0 = callback;
  ALOGI("registering callback");

  // Kill the worker thread if the new callback is NULL.
  if (mCallback_1_0 == NULL) {
    pthread_mutex_unlock(&mLock);
    if (!pthread_kill(mPoll, SIGUSR1)) {
      pthread_join(mPoll, NULL);
      ALOGI("pthread destroyed");
    }
    return Void();
  }

  destroyThread = false;
  signal(SIGUSR1, sighandler);

  /*
   * Create a background thread if the old callback value is NULL
   * and being updated with a new value.
   */
  if (pthread_create(&mPoll, NULL, work, this)) {
    ALOGE("pthread creation failed %d", errno);
    mCallback_1_0 = NULL;
  }

  pthread_mutex_unlock(&mLock);

  checkUsbWakeupSupport(this);

  /*
   * Check for the correct path to detect contaminant presence status
   * from the possible paths and use that to get contaminant
   * presence status when required.
   */
  if (access("/sys/class/power_supply/usb/moisture_detected", R_OK) == 0) {
    mContaminantStatusPath = "/sys/class/power_supply/usb/moisture_detected";
  } else if (access("/sys/class/qcom-battery/moisture_detection_status", R_OK) == 0) {
    mContaminantStatusPath = "/sys/class/qcom-battery/moisture_detection_status";
  } else if (access("/sys/bus/iio/devices/iio:device4/in_index_usb_moisture_detected_input", R_OK) == 0) {
    mContaminantStatusPath = "/sys/bus/iio/devices/iio:device4/in_index_usb_moisture_detected_input";
  } else {
    mContaminantStatusPath.clear();
  }

  ALOGI("Contamination presence path: %s", mContaminantStatusPath.c_str());

  return Void();
}

static void checkUsbWakeupSupport(struct Usb *usb) {
  std::string platdevices = "/sys/bus/platform/devices/";
  DIR *pd = opendir(platdevices.c_str());
  if (pd != NULL) {
    struct dirent *platDir;
    while ((platDir = readdir(pd))) {
      std::string cname = platDir->d_name;
      /*
       * Scan for USB controller. Here "susb" takes care of both hsusb and ssusb.
       * Set mIgnoreWakeup based on the availability of 1st Controller's
       * power/wakeup node.
       */
      if (strstr(platDir->d_name, "susb")) {
	if (faccessat(dirfd(pd), (cname + "/power/wakeup").c_str(), F_OK, 0) < 0) {
	  usb->mIgnoreWakeup = true;
	  ALOGI("PLATFORM DOESN'T SUPPORT WAKEUP");
	} else {
	  usb->mIgnoreWakeup = false;
	}
	break;
      }
    }
    closedir(pd);
  }

  if (usb->mIgnoreWakeup)
    return;

  /*
   * If wakeup is supported then scan for enumerated USB devices and
   * enable autosuspend.
   */
  std::string usbdevices = "/sys/bus/usb/devices/";
  DIR *dp = opendir(usbdevices.c_str());
  if (dp != NULL) {
    struct dirent *deviceDir;
    struct dirent *intfDir;
    DIR *ip;

    while ((deviceDir = readdir(dp))) {
      /*
       * Iterate over all the devices connected over USB while skipping
       * the interfaces.
       */
      if (deviceDir->d_type == DT_LNK && !strchr(deviceDir->d_name, ':')) {
        char buf[PATH_MAX];
        if (realpath((usbdevices + deviceDir->d_name).c_str(), buf)) {

          ip = opendir(buf);
          if (ip == NULL)
            continue;

          while ((intfDir = readdir(ip))) {
            // Scan over all the interfaces that are part of the device
            if (intfDir->d_type == DT_DIR && strchr(intfDir->d_name, ':')) {
              /*
               * If the autosuspend is successfully enabled, no need
               * to iterate over other interfaces.
               */
              if (checkUsbInterfaceAutoSuspend(buf, intfDir->d_name))
                break;
            }
          }
          closedir(ip);
        }
      }
    }
    closedir(dp);
  }
}

/*
 * allow specific USB device idProduct and idVendor to auto suspend
 */
static bool canProductAutoSuspend(const std::string &deviceIdVendor,
    const std::string &deviceIdProduct) {
  if (deviceIdVendor == GOOGLE_USB_VENDOR_ID_STR &&
      deviceIdProduct == GOOGLE_USBC_35_ADAPTER_UNPLUGGED_ID_STR) {
    return true;
  }
  return false;
}

static bool canUsbDeviceAutoSuspend(const std::string &devicePath) {
  std::string deviceIdVendor;
  std::string deviceIdProduct;
  readFile(devicePath + "/idVendor", &deviceIdVendor);
  readFile(devicePath + "/idProduct", &deviceIdProduct);

  // deviceIdVendor and deviceIdProduct will be empty strings if readFile fails
  return canProductAutoSuspend(deviceIdVendor, deviceIdProduct);
}

/*
 * function to consume USB device plugin events (on receiving a
 * USB device path string), and enable autosupend on the USB device if
 * necessary.
 */
static void checkUsbDeviceAutoSuspend(const std::string& devicePath) {
  /*
   * Currently we only actively enable devices that should be autosuspended, and leave others
   * to the defualt.
   */
  if (canUsbDeviceAutoSuspend(devicePath)) {
    ALOGI("auto suspend usb device %s", devicePath.c_str());
    writeFile(devicePath + "/power/control", "auto");
    writeFile(devicePath + "/power/wakeup", "enabled");
  }
}

static bool checkUsbInterfaceAutoSuspend(const std::string& devicePath,
        const std::string &intf) {
  std::string bInterfaceClass;
  int interfaceClass, ret = -1;

  readFile(devicePath + "/" + intf + "/bInterfaceClass", &bInterfaceClass);
  interfaceClass = std::stoi(bInterfaceClass, 0, 16);

  // allow autosuspend for certain class devices
  switch (interfaceClass) {
    case USB_CLASS_AUDIO:
    case USB_CLASS_HUB:
      ALOGI("auto suspend usb interfaces %s", devicePath.c_str());
      ret = writeFile(devicePath + "/power/control", "auto");
      if (ret)
        break;

      ret = writeFile(devicePath + "/power/wakeup", "enabled");
      break;
     default:
      ALOGI("usb interface does not support autosuspend %s", devicePath.c_str());

  }

  return ret ? false : true;
}

}  // namespace implementation
}  // namespace V1_3
}  // namespace usb
}  // namespace hardware
}  // namespace android

int main() {
  using android::base::GetProperty;
  using android::hardware::configureRpcThreadpool;
  using android::hardware::joinRpcThreadpool;
  using android::hardware::usb::V1_3::IUsb;
  using android::hardware::usb::V1_3::implementation::Usb;

  android::sp<IUsb> service = new Usb(
      GetProperty(USB_DEVICE_PROP, "4e00000.ssusb"),
      GetProperty(USB_CONTROLLER_PROP, "4e00000.dwc3"));

  configureRpcThreadpool(1, true /*callerWillJoin*/);
  android::status_t status = service->registerAsService();

  if (status != android::OK) {
    ALOGE("Cannot register USB HAL service");
    return 1;
  }

  ALOGI("QTI USB HAL Ready.");
  joinRpcThreadpool();
  // Under normal cases, execution will not reach this line.
  ALOGI("QTI USB HAL failed to join thread pool.");
  return 1;
}
