#pragma once

#include <vector>
#include <deque>
#include <cstdint>
#include <memory>
#include <map>
#include <set>
#include <functional>
#include <utility>
#include <initializer_list>

#ifndef ARDUINOCORE_API_HARDWARECAN_H
#error "CANDemux.h: include a HardwareCAN provider (e.g. <Arduino_CAN.h>, <ESP32_TWAI.h>, <RP2040PIO_CAN.h>) before this file."
#endif

class CANDemux;
class VirtualCANImpl;

class VirtualCAN : public arduino::HardwareCAN {
public:
  VirtualCAN(VirtualCANImpl *client = nullptr);

  bool begin(CanBitRate const can_bitrate) override;
  void end() noexcept override;
  int write(const CanMsg &msg) override;
  size_t available() noexcept override;
  CanMsg read() noexcept override;

  void addId(uint32_t id);
  void addRange(uint32_t start, uint32_t count);

  enum class OverflowPolicy {
    DropNewest,
    DropOldest
  };
  void setOverflowPolicy(OverflowPolicy policy);
  void onQueueOverflow(std::function<void()> callback);

private:
  VirtualCANImpl *client_;
};

class VirtualCANImpl : public arduino::HardwareCAN {
public:
  using OverflowPolicy = VirtualCAN::OverflowPolicy;

  explicit VirtualCANImpl(CANDemux *hub, size_t queue_size) noexcept
    : hub_(hub), max_queue_size_(queue_size) {}

  bool begin(CanBitRate const can_bitrate) override;
  void end() noexcept override;
  int write(const CanMsg &msg) override;
  size_t available() noexcept override;
  CanMsg read() noexcept override;

  void addId(uint32_t id);
  void addRange(uint32_t start, uint32_t count);

  void setOverflowPolicy(OverflowPolicy policy) {
    overflow_policy_ = policy;
  }
  void onQueueOverflow(std::function<void()> callback) {
    overflow_callback_ = callback;
  }

private:
  friend class CANDemux;
  bool matchesRange(uint32_t id) const noexcept {
    for (const auto &r : ranges_) {
      if (id >= r.first && id < r.second)
        return true;
    }
    return false;
  }

  void push(const CanMsg &msg) {
    if (rxq_.size() < max_queue_size_) {
      rxq_.push_back(msg);
    } else {
      if (overflow_callback_) {
        overflow_callback_();
      }
      if (overflow_policy_ == OverflowPolicy::DropOldest) {
        rxq_.pop_front();
        rxq_.push_back(msg);
      }
      // DropNewest (default) の場合は何もしない
    }
  }

  CANDemux *hub_ = nullptr;
  std::vector<std::pair<uint32_t, uint32_t>> ranges_;
  std::deque<CanMsg> rxq_;
  size_t max_queue_size_;
  OverflowPolicy overflow_policy_ = OverflowPolicy::DropNewest;
  std::function<void()> overflow_callback_ = nullptr;
};

class CANDemux {
public:
  explicit CANDemux(arduino::HardwareCAN *base) noexcept : base_(base) {}

  CANDemux(const CANDemux &) = delete;
  CANDemux &operator=(const CANDemux &) = delete;

  VirtualCAN createClient(size_t queue_size = 8) {
    clients_.emplace_back(std::make_unique<VirtualCANImpl>(this, queue_size));
    return VirtualCAN(clients_.back().get());
  }

  template<typename T>
  VirtualCAN createClientWithIds(const T &ids, size_t queue_size = 8) {
    VirtualCAN c = createClient(queue_size);
    for (const uint32_t id : ids) {
      c.addId(id);
    }
    return c;
  }

  // リスト指定: ID リストを完全一致で受信対象として登録
  // 例: `{ 0x100, 0x200 }`
  VirtualCAN createClientWithIds(std::initializer_list<uint32_t> ids, size_t queue_size = 8) {
    VirtualCAN c = createClient(queue_size);
    for (auto id : ids)
      c.addId(id);
    return c;
  }

  // 範囲指定: start と count による半開区間 `[start, start+count)` を受信対象として登録
  // 例: `start=0x100, count=4 -> 0x100, 0x101, 0x102, 0x103`
  VirtualCAN createClientWithRange(uint32_t start, uint32_t count, size_t queue_size = 8) {
    VirtualCAN c = createClient(queue_size);
    c.addRange(start, count);
    return c;
  }

  void poll() {
    if (base_ == nullptr)
      return;

    while (base_->available()) {
      CanMsg m = base_->read();
      const uint32_t id = m.id;

      std::set<VirtualCANImpl *> recipients;

      auto it = id_subscriptions_.find(id);
      if (it != id_subscriptions_.end()) {
        recipients.insert(it->second.begin(), it->second.end());
      }

      for (VirtualCANImpl *c : range_subscription_clients_) {
        if (c->matchesRange(id)) {
          recipients.insert(c);
        }
      }

      for (VirtualCANImpl *c : recipients) {
        c->push(m);
      }
    }
  }

  int write(const CanMsg &msg) {
    if (base_ == nullptr)
      return -1;
    return base_->write(msg);
  }

private:
  friend class VirtualCANImpl;
  void subscribeId(uint32_t id, VirtualCANImpl *client) {
    id_subscriptions_[id].push_back(client);
  }

  void subscribeRange(VirtualCANImpl *client) {
    range_subscription_clients_.insert(client);
  }

  arduino::HardwareCAN *base_ = nullptr;
  std::vector<std::unique_ptr<VirtualCANImpl>> clients_;
  std::map<uint32_t, std::vector<VirtualCANImpl *>> id_subscriptions_;
  std::set<VirtualCANImpl *> range_subscription_clients_;
};

inline VirtualCAN::VirtualCAN(VirtualCANImpl *client) : client_(client) {}
inline bool VirtualCAN::begin(CanBitRate const can_bitrate) {
  return client_ ? client_->begin(can_bitrate) : false;
}
inline void VirtualCAN::end() noexcept {
  if (client_)
    client_->end();
}
inline int VirtualCAN::write(const CanMsg &msg) {
  return client_ ? client_->write(msg) : -1;
}
inline size_t VirtualCAN::available() noexcept {
  return client_ ? client_->available() : 0;
}
inline CanMsg VirtualCAN::read() noexcept {
  return client_ ? client_->read() : CanMsg{};
}
inline void VirtualCAN::addId(uint32_t id) {
  if (client_)
    client_->addId(id);
}
inline void VirtualCAN::addRange(uint32_t start, uint32_t count) {
  if (client_)
    client_->addRange(start, count);
}
inline void VirtualCAN::setOverflowPolicy(OverflowPolicy policy) {
  if (client_)
    client_->setOverflowPolicy(policy);
}
inline void VirtualCAN::onQueueOverflow(std::function<void()> callback) {
  if (client_)
    client_->onQueueOverflow(callback);
}

inline void VirtualCANImpl::addId(uint32_t id) {
  if (hub_)
    hub_->subscribeId(id, this);
}

inline void VirtualCANImpl::addRange(uint32_t start, uint32_t count) {
  ranges_.emplace_back(start, start + count);
  if (hub_)
    hub_->subscribeRange(this);
}

inline bool VirtualCANImpl::begin(CanBitRate const can_bitrate) {
  (void)can_bitrate;  // "unused parameter" 警告を抑制
  return hub_ != nullptr;
}

inline void VirtualCANImpl::end() noexcept {
  // no-op
}

inline int VirtualCANImpl::write(const CanMsg &msg) {
  if (!hub_)
    return -1;
  return hub_->write(msg);
}

inline size_t VirtualCANImpl::available() noexcept {
  if (hub_)
    hub_->poll();
  return rxq_.size();
}

inline CanMsg VirtualCANImpl::read() noexcept {
  if (rxq_.empty())
    return CanMsg{};
  CanMsg m = std::move(rxq_.front());
  rxq_.pop_front();
  return m;
}
