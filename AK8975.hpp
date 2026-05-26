#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: AK8975 magnetometer driver module
constructor_args:
  - rotation:
      w: 1.0
      x: 0.0
      y: 0.0
      z: 0.0
  - data_topic_name: "ak8975_mag"
  - sample_period_ms: 20
  - task_stack_depth: 1024
template_args: []
required_hardware:
  - spi_ak8975/spi2/SPI2
  - ak8975_cs
  - spi2_mutex
  - ramfs
depends: []
=== END MANIFEST === */
// clang-format on

#include "app_framework.hpp"
#include "gpio.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "ramfs.hpp"
#include "spi.hpp"
#include "thread.hpp"
#include "transform.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include <atomic>

#ifndef XR_STDIO_PRINTF_COMPAT
#if __has_include("print.hpp")
#define XR_STDIO_PRINTF_COMPAT(fmt, ...) LibXR::STDIO::Printf<fmt>(__VA_ARGS__)
#else
#define XR_STDIO_PRINTF_COMPAT(fmt, ...) \
  LibXR::STDIO::Printf(fmt __VA_OPT__(, ) __VA_ARGS__)
#endif
#endif

class AK8975 : public LibXR::Application {
 public:
  class OptionalBusLock {
   public:
    explicit OptionalBusLock(LibXR::Mutex* mutex) : mutex_(mutex) {
      if (mutex_ != nullptr) {
        mutex_->Lock();
      }
    }

    ~OptionalBusLock() {
      if (mutex_ != nullptr) {
        mutex_->Unlock();
      }
    }

   private:
    LibXR::Mutex* mutex_;
  };

  AK8975(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
         LibXR::Quaternion<float>&& rotation, const char* data_topic_name,
         uint32_t sample_period_ms, size_t task_stack_depth)
      : sample_period_ms_(sample_period_ms),
        topic_(data_topic_name, sizeof(mag_data_)),
        cs_(hw.template FindOrExit<LibXR::GPIO>({"ak8975_cs"})),
        spi_(hw.template FindOrExit<LibXR::SPI>({"spi_ak8975", "spi2", "SPI2"})),
        spi_mutex_(hw.template Find<LibXR::Mutex>({"spi2_mutex"})),
        rotation_(std::move(rotation)),
        op_spi_(sem_spi_),
        cmd_file_(LibXR::RamFS::CreateFile("ak8975", CommandFunc, this)) {
    app.Register(*this);
    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);

    ASSERT(spi_->SetConfig({.clock_polarity = LibXR::SPI::ClockPolarity::HIGH,
                            .clock_phase = LibXR::SPI::ClockPhase::EDGE_2,
                            .prescaler = LibXR::SPI::Prescaler::DIV_4}) ==
           LibXR::ErrorCode::OK);

    cs_->SetConfig({.direction = LibXR::GPIO::Direction::OUTPUT_PUSH_PULL,
                    .pull = LibXR::GPIO::Pull::NONE});
    cs_->Write(true);

    chip_id_ = ReadReg(REG_WIA);
    ASSERT(chip_id_ == 0x48);

    TriggerMeasurement();

    XR_LOG_PASS("AK8975: Init succeeded.");

    thread_.Create(this, ThreadFunc, "ak8975_thread", task_stack_depth,
                   LibXR::Thread::Priority::HIGH);
  }

  void OnMonitor() override {
    if (std::isnan(mag_data_.x()) || std::isnan(mag_data_.y()) ||
        std::isnan(mag_data_.z())) {
      XR_LOG_WARN("AK8975: NaN data detected.");
    }
  }

  void RequestMagCalibration() {
    mag_cali_requested_.store(true, std::memory_order_release);
  }

 private:
  static constexpr uint8_t REG_WIA = 0x00;
  static constexpr uint8_t REG_HXL = 0x03;
  static constexpr uint8_t REG_CNTL = 0x0A;

  void WriteReg(uint8_t reg, uint8_t value) {
    OptionalBusLock lock(spi_mutex_);
    cs_->Write(false);
    spi_->MemWrite(reg, value, op_spi_);
    cs_->Write(true);
  }

  uint8_t ReadReg(uint8_t reg) {
    OptionalBusLock lock(spi_mutex_);
    uint8_t value = 0;
    cs_->Write(false);
    spi_->MemRead(reg, {&value, 1}, op_spi_);
    cs_->Write(true);
    return value;
  }

  void ReadRegs(uint8_t reg, uint8_t* data, size_t size) {
    OptionalBusLock lock(spi_mutex_);
    cs_->Write(false);
    spi_->MemRead(reg, {data, size}, op_spi_);
    cs_->Write(true);
  }

  void TriggerMeasurement() { WriteReg(REG_CNTL, 0x01); }

  void BeginCalibration() {
    calibrating_ = true;
    cali_end_time_ms_ = LibXR::Thread::GetTime() + 15000;
    cali_min_.fill(INT16_MAX);
    cali_max_.fill(INT16_MIN);
    XR_LOG_PASS("AK8975: Mag calibration started.");
  }

  void Update() {
    uint8_t raw[6] = {0};
    ReadRegs(REG_HXL, raw, sizeof(raw));

    int16_t tmp_x = static_cast<int16_t>((static_cast<uint16_t>(raw[1]) << 8) | raw[0]);
    int16_t tmp_y = static_cast<int16_t>((static_cast<uint16_t>(raw[3]) << 8) | raw[2]);
    int16_t tmp_z = static_cast<int16_t>((static_cast<uint16_t>(raw[5]) << 8) | raw[4]);

    Eigen::Matrix<float, 3, 1> mapped;
    mapped << static_cast<float>(-tmp_x), static_cast<float>(tmp_y),
        static_cast<float>(-tmp_z);
    if (calibrating_) {
      for (int i = 0; i < 3; ++i) {
        cali_min_[i] = std::min(cali_min_[i], static_cast<int16_t>(mapped[i]));
        cali_max_[i] = std::max(cali_max_[i], static_cast<int16_t>(mapped[i]));
      }

      if (LibXR::Thread::GetTime() >= cali_end_time_ms_) {
        Eigen::Matrix<float, 3, 1> range;
        range << static_cast<float>(cali_max_[0] - cali_min_[0]),
            static_cast<float>(cali_max_[1] - cali_min_[1]),
            static_cast<float>(cali_max_[2] - cali_min_[2]);
        const float avg_radius = (range[0] + range[1] + range[2]) / 3.0f;
        for (int i = 0; i < 3; ++i) {
          offset_[i] =
              static_cast<float>(cali_max_[i] + cali_min_[i]) * 0.5f;
          scale_[i] = range[i] > 1.0f ? avg_radius / range[i] : 1.0f;
        }
        calibrating_ = false;
        XR_LOG_PASS("AK8975: Mag calibration finished.");
      }
    }

    mapped -= offset_;
    mapped[0] *= scale_[0];
    mapped[1] *= scale_[1];
    mapped[2] *= scale_[2];
    mag_data_ = rotation_ * mapped;

    TriggerMeasurement();
  }

  void CheckCalibrationRequest() {
    if (mag_cali_requested_.exchange(false, std::memory_order_acq_rel)) {
      BeginCalibration();
    }
  }

  static void ThreadFunc(AK8975* ak8975) {
    while (true) {
      ak8975->CheckCalibrationRequest();
      ak8975->Update();
      ak8975->topic_.Publish(ak8975->mag_data_);
      LibXR::Thread::Sleep(ak8975->sample_period_ms_);
    }
  }

  static int CommandFunc(AK8975* ak8975, int argc, char** argv) {
    if (argc == 1) {
      XR_STDIO_PRINTF_COMPAT("Usage:\r\n");
      XR_STDIO_PRINTF_COMPAT(
          "  show [time_ms] [interval_ms] - Print magnetometer data periodically.\r\n");
      return 0;
    }

    if (argc == 4 && std::strcmp(argv[1], "show") == 0) {
      int time_ms = std::atoi(argv[2]);
      int interval_ms = std::atoi(argv[3]);
      interval_ms = std::clamp(interval_ms, 10, 1000);

      while (time_ms > 0) {
        XR_STDIO_PRINTF_COMPAT("AK8975: x=%f y=%f z=%f\r\n",
                               ak8975->mag_data_.x(), ak8975->mag_data_.y(),
                               ak8975->mag_data_.z());
        LibXR::Thread::Sleep(interval_ms);
        time_ms -= interval_ms;
      }
      return 0;
    }

    XR_STDIO_PRINTF_COMPAT("Error: Invalid arguments.\r\n");
    return -1;
  }

  uint32_t sample_period_ms_ = 20;
  uint8_t chip_id_ = 0;
  Eigen::Matrix<float, 3, 1> mag_data_ = Eigen::Matrix<float, 3, 1>(0.0f, 0.0f, 0.0f);
  LibXR::Topic topic_;
  LibXR::GPIO* cs_;
  LibXR::SPI* spi_;
  LibXR::Mutex* spi_mutex_ = nullptr;
  LibXR::Quaternion<float> rotation_;
  bool calibrating_ = false;
  std::atomic<bool> mag_cali_requested_{false};
  uint32_t cali_end_time_ms_ = 0;
  std::array<int16_t, 3> cali_min_ = {INT16_MAX, INT16_MAX, INT16_MAX};
  std::array<int16_t, 3> cali_max_ = {INT16_MIN, INT16_MIN, INT16_MIN};
  Eigen::Matrix<float, 3, 1> offset_ = Eigen::Matrix<float, 3, 1>(0.0f, 0.0f, 0.0f);
  Eigen::Matrix<float, 3, 1> scale_ = Eigen::Matrix<float, 3, 1>(1.0f, 1.0f, 1.0f);
  LibXR::Semaphore sem_spi_;
  LibXR::SPI::OperationRW op_spi_;
  LibXR::RamFS::File cmd_file_;
  LibXR::Thread thread_;
};
