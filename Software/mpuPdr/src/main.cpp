#include <Arduino.h>
#include "WiFi.h"
#include "MPU9250.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h"
#include "sdkconfig.h"
#include <ESP32Scheduler.h>
#include <Task.h>

#define MPU9250_IMU_ADDRESS 0x68
#define MAGNETIC_DECLINATION 4.71666667
#define INTERVAL_MS_PRINT 1000

#define YAW_CALIBRATION 113.4
#define PITCH_CALIBRATION 2.4
#define ROLL_CALIBRATION 179.9

#define ACC_X_CALIBRATION 0.05
#define ACC_Y_CALIBRATION 0.01
#define ACC_Z_CALIBRATION 1.02

#define MAG_X_CALIBRATION 0
#define MAG_Y_CALIBRATION 0
#define MAG_Z_CALIBRATION 0

#define IMU_GYR_X_IND 0
#define IMU_GYR_Y_IND 1
#define IMU_GYR_Z_IND 2
#define IMU_ACC_X_IND 3
#define IMU_ACC_Y_IND 4
#define IMU_ACC_Z_IND 5
#define IMU_MAG_X_IND 6
#define IMU_MAG_Y_IND 7
#define IMU_MAG_Z_IND 8
#define IMU_TEMP_IND 9


String cardinal;
MPU9250 mpu;


double imuData[10] = {0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 , 0};

unsigned long lastPrintMillis = 0;

static uint64_t idle0Calls = 0;
static uint64_t idle1Calls = 0;
static const uint64_t MaxIdleCalls = 1855000;

const char* ssid = "Saba";
const char* password = "Saba1115";
const char* MyHostName = "HealthTracker";


static bool idle_task_0() {
	idle0Calls += 1;
	return false;
}

static bool idle_task_1() {
	idle1Calls += 1;
	return false;
}

void calcDiraction(void) {
  float headingRadians = atan2(imuData[IMU_MAG_Y_IND], imuData[IMU_MAG_X_IND]);
  float headingDegrees = headingRadians * 180 / PI;
  float declinationAngle = MAGNETIC_DECLINATION;

  headingDegrees += declinationAngle;

  if (headingDegrees < 0) {
      headingDegrees += 360;
  }

  if (headingDegrees > 348.75 || headingDegrees < 11.25) {
      cardinal = " N";
  }
  else if (headingDegrees > 11.25 && headingDegrees < 33.75) {
      cardinal = " NE"; //NNE
  }
  else if (headingDegrees > 33.75 && headingDegrees < 56.25) {
      cardinal = " NE";
  }
  else if (headingDegrees > 56.25 && headingDegrees < 78.75) {
      cardinal = " NE"; //ENE
  }
  else if (headingDegrees > 78.75 && headingDegrees < 101.25) {
      cardinal = " E";
  }
  else if (headingDegrees > 101.25 && headingDegrees < 123.75) {
      cardinal = " SE"; //ESE
  }
  else if (headingDegrees > 123.75 && headingDegrees < 146.25) {
      cardinal = " SE";
  }
  else if (headingDegrees > 146.25 && headingDegrees < 168.75) {
      cardinal = " SE"; //SSE
  }
  else if (headingDegrees > 168.75 && headingDegrees < 191.25) {
      cardinal = " S";
  }
  else if (headingDegrees > 191.25 && headingDegrees < 213.75) {
      cardinal = " SW"; //SSW
  }
  else if (headingDegrees > 213.75 && headingDegrees < 236.25) {
      cardinal = " SW";
  }
  else if (headingDegrees > 236.25 && headingDegrees < 258.75) {
      cardinal = " SW"; //WSW
  }
  else if (headingDegrees > 258.75 && headingDegrees < 281.25) {
      cardinal = " W";
  }
  else if (headingDegrees > 281.25 && headingDegrees < 303.75) {
      cardinal = " NW"; //WNW
  }
  else if (headingDegrees > 303.75 && headingDegrees < 326.25) {
      cardinal = " NW";
  }
  else if (headingDegrees > 326.25 && headingDegrees < 348.75) {
      cardinal = " NW"; //NNW
  }

  Serial.print("Heading: ");
  Serial.print(headingDegrees);
  Serial.println(cardinal);

  Serial.println("");
}

void imuGetData(void) {
  imuData[IMU_TEMP_IND] = mpu.getTemperature();
  Serial.print("TEMP:\t");
  Serial.println(imuData[IMU_TEMP_IND], 2);

  imuData[IMU_GYR_X_IND] = mpu.getYaw();
  imuData[IMU_GYR_Y_IND] = mpu.getPitch();
  imuData[IMU_GYR_Z_IND] = mpu.getRoll();
  Serial.print("Yaw: ");   Serial.println(imuData[IMU_GYR_X_IND], 2);
  Serial.print("Pitch: "); Serial.println(imuData[IMU_GYR_Y_IND], 2);
  Serial.print("Roll: ");  Serial.println(imuData[IMU_GYR_Z_IND], 2);

  imuData[IMU_ACC_X_IND] = mpu.getAccX();
  imuData[IMU_ACC_Y_IND] = mpu.getAccY();
  imuData[IMU_ACC_Z_IND] = mpu.getAccZ();
  Serial.print("ACC_X: "); Serial.println(imuData[IMU_ACC_X_IND], 2);
  Serial.print("ACC_Y: "); Serial.println(imuData[IMU_ACC_Y_IND], 2);
  Serial.print("ACC_Z: "); Serial.println(imuData[IMU_ACC_Z_IND], 2);

  imuData[IMU_MAG_X_IND] = mpu.getMagX();
  imuData[IMU_MAG_Y_IND] = mpu.getMagY();
  imuData[IMU_MAG_Z_IND] = mpu.getMagZ();
  Serial.print("MAG_X: "); Serial.println(imuData[IMU_MAG_X_IND] , 2);
  Serial.print("MAG_Y: "); Serial.println(imuData[IMU_MAG_Y_IND] , 2);
  Serial.print("MAG_Z: "); Serial.println(imuData[IMU_MAG_Z_IND] , 2);

  Serial.println("");
}

void print_calibration() {
    // Serial.println("< calibration parameters >");
    // Serial.println("accel bias [g]: ");
    // Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    // Serial.print(", ");
    // Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    // Serial.print(", ");
    // Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    // Serial.println();
    // Serial.println("gyro bias [deg/s]: ");
    // Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    // Serial.print(", ");
    // Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    // Serial.print(", ");
    // Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    // Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}

double acc_GtoM(double acc) {
  return acc * 9.80665;
}

double acc_vectorial(double acc_x ,double acc_y) {
  return atan2(acc_x ,acc_y);
}


class IMU : public Task {
protected:
    void setup() {
      // MPU9250Setting setting;
      // Sample rate must be at least 2x DLPF rate
      // setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
      // setting.gyro_fs_sel = GYRO_FS_SEL::G1000DPS;
      // setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
      // setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_250HZ;
      // setting.gyro_fchoice = 0x03;
      // setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_20HZ;
      // setting.accel_fchoice = 0x01;
      // setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

      // mpu.setup(MPU9250_IMU_ADDRESS, setting);
      mpu.setup(MPU9250_IMU_ADDRESS);

      mpu.setMagneticDeclination(MAGNETIC_DECLINATION);
      // mpu.selectFilter(QuatFilterSel::MADGWICK);
      // mpu.setFilterIterations(15);

      mpu.verbose(true);
      delay(5000);
      mpu.calibrateAccelGyro();

      Serial.println("Mag calibration will start in 5sec.");
      Serial.println("Please Wave device in a figure eight until done.");
      delay(5000);
      mpu.calibrateMag();

      print_calibration();
      mpu.verbose(false);

      wifiTimeStamp = millis();
    }

    void loop() {
      if (mpu.update()) {
        if ((millis() - wifiTimeStamp) > 100) {
            imuGetData();
            calcDiraction();
            wifiTimeStamp = millis();
        }
      }

      vTaskDelay(1);
    }

private:
    unsigned long wifiTimeStamp = 0;
} imu_task;

class CPU : public Task {
protected:
    void setup(){
      ESP_ERROR_CHECK(esp_register_freertos_idle_hook_for_cpu(idle_task_0, 0));
	    ESP_ERROR_CHECK(esp_register_freertos_idle_hook_for_cpu(idle_task_1, 1));
    }
    void loop() {
      float idle0 = idle0Calls;
      float idle1 = idle1Calls;
      idle0Calls = 0;
      idle1Calls = 0;

      int cpu0 = 100.f -  idle0 / MaxIdleCalls * 100.f;
      int cpu1 = 100.f - idle1 / MaxIdleCalls * 100.f;

      Serial.println(String(cpu0) + '%');
      Serial.println(String(cpu1) + '%');
      uint32_t getCpuFrequencyMhz();
      uint32_t getXtalFrequencyMhz();
      uint32_t getApbFrequency();
    }
private:
    uint8_t state;
} cpu_task;

class WIRELESS : public Task {
protected:
    void setup() {
      Serial.begin(115200);

      WiFi.setHostname(MyHostName);

      // Set WiFi to station mode and disconnect from an AP if it was previously connected.
      WiFi.mode(WIFI_STA);
      WiFi.disconnect();
      delay(100);

      WiFi.begin(ssid, password);
      Serial.println("\nConnecting to WiFi Network ..");
      // Wait until connection is established
      while(WiFi.status() != WL_CONNECTED){
          Serial.print(".");
          delay(100);
      }
      // Print ESP32's IP & HostName
      Serial.println("\nConnected to the WiFi network");
      Serial.print("Local ESP32 IP: ");
      Serial.println(WiFi.localIP());
      Serial.print("ESP32 HostName: ");
      Serial.println(WiFi.getHostname());

      timeStamp = millis();
    }

    void loop() {
      // if (WiFi.isConnected() && ((millis() - timeStamp) > 5000)) {
      //   Serial.println("ESP32 Wireless : WiFi is Connected to -> " + String(ssid));
      //   Serial.println();
      //   timeStamp = millis();
      // }
      
      Serial.println("Scan start");
  
      // WiFi.scanNetworks will return the number of networks found.
      int n = WiFi.scanNetworks();
      Serial.println("Scan done");
      if (n == 0) {
          Serial.println("no networks found");
      } else {
          Serial.print(n);
          Serial.println(" networks found");
          Serial.println("Nr | SSID                             | RSSI | CH | Encryption");
          for (int i = 0; i < n; ++i) {
              // Print SSID and RSSI for each network found
              Serial.printf("%2d",i + 1);
              Serial.print(" | ");
              Serial.printf("%-32.32s", WiFi.SSID(i).c_str());
              Serial.print(" | ");
              Serial.printf("%4d", WiFi.RSSI(i));
              Serial.print(" | ");
              Serial.printf("%2d", WiFi.channel(i));
              Serial.print(" | ");
              switch (WiFi.encryptionType(i))
              {
              case WIFI_AUTH_OPEN:
                  Serial.print("open");
                  break;
              case WIFI_AUTH_WEP:
                  Serial.print("WEP");
                  break;
              case WIFI_AUTH_WPA_PSK:
                  Serial.print("WPA");
                  break;
              case WIFI_AUTH_WPA2_PSK:
                  Serial.print("WPA2");
                  break;
              case WIFI_AUTH_WPA_WPA2_PSK:
                  Serial.print("WPA+WPA2");
                  break;
              case WIFI_AUTH_WPA2_ENTERPRISE:
                  Serial.print("WPA2-EAP");
                  break;
              case WIFI_AUTH_WPA3_PSK:
                  Serial.print("WPA3");
                  break;
              case WIFI_AUTH_WPA2_WPA3_PSK:
                  Serial.print("WPA2+WPA3");
                  break;
              case WIFI_AUTH_WAPI_PSK:
                  Serial.print("WAPI");
                  break;
              default:
                  Serial.print("unknown");
              }
              Serial.println();
              vTaskDelay(10);
          }
      }
      Serial.println("");
  
      // Delete the scan result to free memory for code below.
      WiFi.scanDelete();
  
      vTaskDelay(1);
    }
private:
    unsigned long timeStamp = 0;
} wireless_task;


void setup() {
  Serial.begin(115200);
  Wire.begin(10 ,9);
  
//   Scheduler.start(&imu_task);
  // Scheduler.start(&cpu_task);
  Scheduler.start(&wireless_task);
  Scheduler.begin();
}

void loop() {

}




