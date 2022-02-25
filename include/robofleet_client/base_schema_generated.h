// automatically generated by the FlatBuffers compiler, do not modify


#ifndef FLATBUFFERS_GENERATED_BASESCHEMA_FB_H_
#define FLATBUFFERS_GENERATED_BASESCHEMA_FB_H_

#include "flatbuffers/flatbuffers.h"

namespace fb {

struct MsgMetadata;
struct MsgMetadataBuilder;
struct MsgMetadataT;

struct MsgWithMetadata;
struct MsgWithMetadataBuilder;
struct MsgWithMetadataT;

struct RosTime;

struct RosDuration;

struct RobofleetSubscription;
struct RobofleetSubscriptionBuilder;
struct RobofleetSubscriptionT;

FLATBUFFERS_MANUALLY_ALIGNED_STRUCT(4) RosTime FLATBUFFERS_FINAL_CLASS {
 private:
  uint32_t sec_;
  uint32_t nsec_;

 public:
  RosTime()
      : sec_(0),
        nsec_(0) {
  }
  RosTime(uint32_t _sec, uint32_t _nsec)
      : sec_(flatbuffers::EndianScalar(_sec)),
        nsec_(flatbuffers::EndianScalar(_nsec)) {
  }
  uint32_t sec() const {
    return flatbuffers::EndianScalar(sec_);
  }
  uint32_t nsec() const {
    return flatbuffers::EndianScalar(nsec_);
  }
};
FLATBUFFERS_STRUCT_END(RosTime, 8);

FLATBUFFERS_MANUALLY_ALIGNED_STRUCT(4) RosDuration FLATBUFFERS_FINAL_CLASS {
 private:
  int32_t sec_;
  int32_t nsec_;

 public:
  RosDuration()
      : sec_(0),
        nsec_(0) {
  }
  RosDuration(int32_t _sec, int32_t _nsec)
      : sec_(flatbuffers::EndianScalar(_sec)),
        nsec_(flatbuffers::EndianScalar(_nsec)) {
  }
  int32_t sec() const {
    return flatbuffers::EndianScalar(sec_);
  }
  int32_t nsec() const {
    return flatbuffers::EndianScalar(nsec_);
  }
};
FLATBUFFERS_STRUCT_END(RosDuration, 8);

struct MsgMetadataT : public flatbuffers::NativeTable {
  typedef MsgMetadata TableType;
  std::string type{};
  std::string topic{};
};

struct MsgMetadata FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
  typedef MsgMetadataT NativeTableType;
  typedef MsgMetadataBuilder Builder;
  enum FlatBuffersVTableOffset FLATBUFFERS_VTABLE_UNDERLYING_TYPE {
    VT_TYPE = 4,
    VT_TOPIC = 6
  };
  const flatbuffers::String *type() const {
    return GetPointer<const flatbuffers::String *>(VT_TYPE);
  }
  const flatbuffers::String *topic() const {
    return GetPointer<const flatbuffers::String *>(VT_TOPIC);
  }
  bool Verify(flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyOffset(verifier, VT_TYPE) &&
           verifier.VerifyString(type()) &&
           VerifyOffset(verifier, VT_TOPIC) &&
           verifier.VerifyString(topic()) &&
           verifier.EndTable();
  }
  MsgMetadataT *UnPack(const flatbuffers::resolver_function_t *_resolver = nullptr) const;
  void UnPackTo(MsgMetadataT *_o, const flatbuffers::resolver_function_t *_resolver = nullptr) const;
  static flatbuffers::Offset<MsgMetadata> Pack(flatbuffers::FlatBufferBuilder &_fbb, const MsgMetadataT* _o, const flatbuffers::rehasher_function_t *_rehasher = nullptr);
};

struct MsgMetadataBuilder {
  typedef MsgMetadata Table;
  flatbuffers::FlatBufferBuilder &fbb_;
  flatbuffers::uoffset_t start_;
  void add_type(flatbuffers::Offset<flatbuffers::String> type) {
    fbb_.AddOffset(MsgMetadata::VT_TYPE, type);
  }
  void add_topic(flatbuffers::Offset<flatbuffers::String> topic) {
    fbb_.AddOffset(MsgMetadata::VT_TOPIC, topic);
  }
  explicit MsgMetadataBuilder(flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  flatbuffers::Offset<MsgMetadata> Finish() {
    const auto end = fbb_.EndTable(start_);
    auto o = flatbuffers::Offset<MsgMetadata>(end);
    return o;
  }
};

inline flatbuffers::Offset<MsgMetadata> CreateMsgMetadata(
    flatbuffers::FlatBufferBuilder &_fbb,
    flatbuffers::Offset<flatbuffers::String> type = 0,
    flatbuffers::Offset<flatbuffers::String> topic = 0) {
  MsgMetadataBuilder builder_(_fbb);
  builder_.add_topic(topic);
  builder_.add_type(type);
  return builder_.Finish();
}

inline flatbuffers::Offset<MsgMetadata> CreateMsgMetadataDirect(
    flatbuffers::FlatBufferBuilder &_fbb,
    const char *type = nullptr,
    const char *topic = nullptr) {
  auto type__ = type ? _fbb.CreateString(type) : 0;
  auto topic__ = topic ? _fbb.CreateString(topic) : 0;
  return fb::CreateMsgMetadata(
      _fbb,
      type__,
      topic__);
}

flatbuffers::Offset<MsgMetadata> CreateMsgMetadata(flatbuffers::FlatBufferBuilder &_fbb, const MsgMetadataT *_o, const flatbuffers::rehasher_function_t *_rehasher = nullptr);

struct MsgWithMetadataT : public flatbuffers::NativeTable {
  typedef MsgWithMetadata TableType;
  std::unique_ptr<fb::MsgMetadataT> __metadata{};
  MsgWithMetadataT() = default;
  MsgWithMetadataT(const MsgWithMetadataT &o);
  MsgWithMetadataT(MsgWithMetadataT&&) FLATBUFFERS_NOEXCEPT = default;
  MsgWithMetadataT &operator=(MsgWithMetadataT o) FLATBUFFERS_NOEXCEPT;
};

struct MsgWithMetadata FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
  typedef MsgWithMetadataT NativeTableType;
  typedef MsgWithMetadataBuilder Builder;
  enum FlatBuffersVTableOffset FLATBUFFERS_VTABLE_UNDERLYING_TYPE {
    VT___METADATA = 4
  };
  const fb::MsgMetadata *__metadata() const {
    return GetPointer<const fb::MsgMetadata *>(VT___METADATA);
  }
  bool Verify(flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyOffset(verifier, VT___METADATA) &&
           verifier.VerifyTable(__metadata()) &&
           verifier.EndTable();
  }
  MsgWithMetadataT *UnPack(const flatbuffers::resolver_function_t *_resolver = nullptr) const;
  void UnPackTo(MsgWithMetadataT *_o, const flatbuffers::resolver_function_t *_resolver = nullptr) const;
  static flatbuffers::Offset<MsgWithMetadata> Pack(flatbuffers::FlatBufferBuilder &_fbb, const MsgWithMetadataT* _o, const flatbuffers::rehasher_function_t *_rehasher = nullptr);
};

struct MsgWithMetadataBuilder {
  typedef MsgWithMetadata Table;
  flatbuffers::FlatBufferBuilder &fbb_;
  flatbuffers::uoffset_t start_;
  void add___metadata(flatbuffers::Offset<fb::MsgMetadata> __metadata) {
    fbb_.AddOffset(MsgWithMetadata::VT___METADATA, __metadata);
  }
  explicit MsgWithMetadataBuilder(flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  flatbuffers::Offset<MsgWithMetadata> Finish() {
    const auto end = fbb_.EndTable(start_);
    auto o = flatbuffers::Offset<MsgWithMetadata>(end);
    return o;
  }
};

inline flatbuffers::Offset<MsgWithMetadata> CreateMsgWithMetadata(
    flatbuffers::FlatBufferBuilder &_fbb,
    flatbuffers::Offset<fb::MsgMetadata> __metadata = 0) {
  MsgWithMetadataBuilder builder_(_fbb);
  builder_.add___metadata(__metadata);
  return builder_.Finish();
}

flatbuffers::Offset<MsgWithMetadata> CreateMsgWithMetadata(flatbuffers::FlatBufferBuilder &_fbb, const MsgWithMetadataT *_o, const flatbuffers::rehasher_function_t *_rehasher = nullptr);

struct RobofleetSubscriptionT : public flatbuffers::NativeTable {
  typedef RobofleetSubscription TableType;
  std::unique_ptr<fb::MsgMetadataT> __metadata{};
  std::string topic_regex{};
  uint8_t action = 0;
  RobofleetSubscriptionT() = default;
  RobofleetSubscriptionT(const RobofleetSubscriptionT &o);
  RobofleetSubscriptionT(RobofleetSubscriptionT&&) FLATBUFFERS_NOEXCEPT = default;
  RobofleetSubscriptionT &operator=(RobofleetSubscriptionT o) FLATBUFFERS_NOEXCEPT;
};

struct RobofleetSubscription FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
  typedef RobofleetSubscriptionT NativeTableType;
  typedef RobofleetSubscriptionBuilder Builder;
  enum FlatBuffersVTableOffset FLATBUFFERS_VTABLE_UNDERLYING_TYPE {
    VT___METADATA = 4,
    VT_TOPIC_REGEX = 6,
    VT_ACTION = 8
  };
  const fb::MsgMetadata *__metadata() const {
    return GetPointer<const fb::MsgMetadata *>(VT___METADATA);
  }
  const flatbuffers::String *topic_regex() const {
    return GetPointer<const flatbuffers::String *>(VT_TOPIC_REGEX);
  }
  uint8_t action() const {
    return GetField<uint8_t>(VT_ACTION, 0);
  }
  bool Verify(flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyOffset(verifier, VT___METADATA) &&
           verifier.VerifyTable(__metadata()) &&
           VerifyOffset(verifier, VT_TOPIC_REGEX) &&
           verifier.VerifyString(topic_regex()) &&
           VerifyField<uint8_t>(verifier, VT_ACTION, 1) &&
           verifier.EndTable();
  }
  RobofleetSubscriptionT *UnPack(const flatbuffers::resolver_function_t *_resolver = nullptr) const;
  void UnPackTo(RobofleetSubscriptionT *_o, const flatbuffers::resolver_function_t *_resolver = nullptr) const;
  static flatbuffers::Offset<RobofleetSubscription> Pack(flatbuffers::FlatBufferBuilder &_fbb, const RobofleetSubscriptionT* _o, const flatbuffers::rehasher_function_t *_rehasher = nullptr);
};

struct RobofleetSubscriptionBuilder {
  typedef RobofleetSubscription Table;
  flatbuffers::FlatBufferBuilder &fbb_;
  flatbuffers::uoffset_t start_;
  void add___metadata(flatbuffers::Offset<fb::MsgMetadata> __metadata) {
    fbb_.AddOffset(RobofleetSubscription::VT___METADATA, __metadata);
  }
  void add_topic_regex(flatbuffers::Offset<flatbuffers::String> topic_regex) {
    fbb_.AddOffset(RobofleetSubscription::VT_TOPIC_REGEX, topic_regex);
  }
  void add_action(uint8_t action) {
    fbb_.AddElement<uint8_t>(RobofleetSubscription::VT_ACTION, action, 0);
  }
  explicit RobofleetSubscriptionBuilder(flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  flatbuffers::Offset<RobofleetSubscription> Finish() {
    const auto end = fbb_.EndTable(start_);
    auto o = flatbuffers::Offset<RobofleetSubscription>(end);
    return o;
  }
};

inline flatbuffers::Offset<RobofleetSubscription> CreateRobofleetSubscription(
    flatbuffers::FlatBufferBuilder &_fbb,
    flatbuffers::Offset<fb::MsgMetadata> __metadata = 0,
    flatbuffers::Offset<flatbuffers::String> topic_regex = 0,
    uint8_t action = 0) {
  RobofleetSubscriptionBuilder builder_(_fbb);
  builder_.add_topic_regex(topic_regex);
  builder_.add___metadata(__metadata);
  builder_.add_action(action);
  return builder_.Finish();
}

inline flatbuffers::Offset<RobofleetSubscription> CreateRobofleetSubscriptionDirect(
    flatbuffers::FlatBufferBuilder &_fbb,
    flatbuffers::Offset<fb::MsgMetadata> __metadata = 0,
    const char *topic_regex = nullptr,
    uint8_t action = 0) {
  auto topic_regex__ = topic_regex ? _fbb.CreateString(topic_regex) : 0;
  return fb::CreateRobofleetSubscription(
      _fbb,
      __metadata,
      topic_regex__,
      action);
}

flatbuffers::Offset<RobofleetSubscription> CreateRobofleetSubscription(flatbuffers::FlatBufferBuilder &_fbb, const RobofleetSubscriptionT *_o, const flatbuffers::rehasher_function_t *_rehasher = nullptr);

inline MsgMetadataT *MsgMetadata::UnPack(const flatbuffers::resolver_function_t *_resolver) const {
  auto _o = std::unique_ptr<MsgMetadataT>(new MsgMetadataT());
  UnPackTo(_o.get(), _resolver);
  return _o.release();
}

inline void MsgMetadata::UnPackTo(MsgMetadataT *_o, const flatbuffers::resolver_function_t *_resolver) const {
  (void)_o;
  (void)_resolver;
  { auto _e = type(); if (_e) _o->type = _e->str(); }
  { auto _e = topic(); if (_e) _o->topic = _e->str(); }
}

inline flatbuffers::Offset<MsgMetadata> MsgMetadata::Pack(flatbuffers::FlatBufferBuilder &_fbb, const MsgMetadataT* _o, const flatbuffers::rehasher_function_t *_rehasher) {
  return CreateMsgMetadata(_fbb, _o, _rehasher);
}

inline flatbuffers::Offset<MsgMetadata> CreateMsgMetadata(flatbuffers::FlatBufferBuilder &_fbb, const MsgMetadataT *_o, const flatbuffers::rehasher_function_t *_rehasher) {
  (void)_rehasher;
  (void)_o;
  struct _VectorArgs { flatbuffers::FlatBufferBuilder *__fbb; const MsgMetadataT* __o; const flatbuffers::rehasher_function_t *__rehasher; } _va = { &_fbb, _o, _rehasher}; (void)_va;
  auto _type = _o->type.empty() ? 0 : _fbb.CreateString(_o->type);
  auto _topic = _o->topic.empty() ? 0 : _fbb.CreateString(_o->topic);
  return fb::CreateMsgMetadata(
      _fbb,
      _type,
      _topic);
}

inline MsgWithMetadataT::MsgWithMetadataT(const MsgWithMetadataT &o)
      : __metadata((o.__metadata) ? new fb::MsgMetadataT(*o.__metadata) : nullptr) {
}

inline MsgWithMetadataT &MsgWithMetadataT::operator=(MsgWithMetadataT o) FLATBUFFERS_NOEXCEPT {
  std::swap(__metadata, o.__metadata);
  return *this;
}

inline MsgWithMetadataT *MsgWithMetadata::UnPack(const flatbuffers::resolver_function_t *_resolver) const {
  auto _o = std::unique_ptr<MsgWithMetadataT>(new MsgWithMetadataT());
  UnPackTo(_o.get(), _resolver);
  return _o.release();
}

inline void MsgWithMetadata::UnPackTo(MsgWithMetadataT *_o, const flatbuffers::resolver_function_t *_resolver) const {
  (void)_o;
  (void)_resolver;
  { auto _e = __metadata(); if (_e) { if(_o->__metadata) { _e->UnPackTo(_o->__metadata.get(), _resolver); } else { _o->__metadata = std::unique_ptr<fb::MsgMetadataT>(_e->UnPack(_resolver)); } } }
}

inline flatbuffers::Offset<MsgWithMetadata> MsgWithMetadata::Pack(flatbuffers::FlatBufferBuilder &_fbb, const MsgWithMetadataT* _o, const flatbuffers::rehasher_function_t *_rehasher) {
  return CreateMsgWithMetadata(_fbb, _o, _rehasher);
}

inline flatbuffers::Offset<MsgWithMetadata> CreateMsgWithMetadata(flatbuffers::FlatBufferBuilder &_fbb, const MsgWithMetadataT *_o, const flatbuffers::rehasher_function_t *_rehasher) {
  (void)_rehasher;
  (void)_o;
  struct _VectorArgs { flatbuffers::FlatBufferBuilder *__fbb; const MsgWithMetadataT* __o; const flatbuffers::rehasher_function_t *__rehasher; } _va = { &_fbb, _o, _rehasher}; (void)_va;
  auto ___metadata = _o->__metadata ? CreateMsgMetadata(_fbb, _o->__metadata.get(), _rehasher) : 0;
  return fb::CreateMsgWithMetadata(
      _fbb,
      ___metadata);
}

inline RobofleetSubscriptionT::RobofleetSubscriptionT(const RobofleetSubscriptionT &o)
      : __metadata((o.__metadata) ? new fb::MsgMetadataT(*o.__metadata) : nullptr),
        topic_regex(o.topic_regex),
        action(o.action) {
}

inline RobofleetSubscriptionT &RobofleetSubscriptionT::operator=(RobofleetSubscriptionT o) FLATBUFFERS_NOEXCEPT {
  std::swap(__metadata, o.__metadata);
  std::swap(topic_regex, o.topic_regex);
  std::swap(action, o.action);
  return *this;
}

inline RobofleetSubscriptionT *RobofleetSubscription::UnPack(const flatbuffers::resolver_function_t *_resolver) const {
  auto _o = std::unique_ptr<RobofleetSubscriptionT>(new RobofleetSubscriptionT());
  UnPackTo(_o.get(), _resolver);
  return _o.release();
}

inline void RobofleetSubscription::UnPackTo(RobofleetSubscriptionT *_o, const flatbuffers::resolver_function_t *_resolver) const {
  (void)_o;
  (void)_resolver;
  { auto _e = __metadata(); if (_e) { if(_o->__metadata) { _e->UnPackTo(_o->__metadata.get(), _resolver); } else { _o->__metadata = std::unique_ptr<fb::MsgMetadataT>(_e->UnPack(_resolver)); } } }
  { auto _e = topic_regex(); if (_e) _o->topic_regex = _e->str(); }
  { auto _e = action(); _o->action = _e; }
}

inline flatbuffers::Offset<RobofleetSubscription> RobofleetSubscription::Pack(flatbuffers::FlatBufferBuilder &_fbb, const RobofleetSubscriptionT* _o, const flatbuffers::rehasher_function_t *_rehasher) {
  return CreateRobofleetSubscription(_fbb, _o, _rehasher);
}

inline flatbuffers::Offset<RobofleetSubscription> CreateRobofleetSubscription(flatbuffers::FlatBufferBuilder &_fbb, const RobofleetSubscriptionT *_o, const flatbuffers::rehasher_function_t *_rehasher) {
  (void)_rehasher;
  (void)_o;
  struct _VectorArgs { flatbuffers::FlatBufferBuilder *__fbb; const RobofleetSubscriptionT* __o; const flatbuffers::rehasher_function_t *__rehasher; } _va = { &_fbb, _o, _rehasher}; (void)_va;
  auto ___metadata = _o->__metadata ? CreateMsgMetadata(_fbb, _o->__metadata.get(), _rehasher) : 0;
  auto _topic_regex = _o->topic_regex.empty() ? 0 : _fbb.CreateString(_o->topic_regex);
  auto _action = _o->action;
  return fb::CreateRobofleetSubscription(
      _fbb,
      ___metadata,
      _topic_regex,
      _action);
}

}  // namespace fb

#endif  // FLATBUFFERS_GENERATED_BASESCHEMA_FB_H_