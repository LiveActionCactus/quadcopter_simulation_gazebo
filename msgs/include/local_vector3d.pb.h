// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: local_vector3d.proto

#ifndef PROTOBUF_local_5fvector3d_2eproto__INCLUDED
#define PROTOBUF_local_5fvector3d_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3000000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3000000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void protobuf_AddDesc_local_5fvector3d_2eproto();
void protobuf_AssignDesc_local_5fvector3d_2eproto();
void protobuf_ShutdownFile_local_5fvector3d_2eproto();

class LocalVector3d;

// ===================================================================

class LocalVector3d : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.LocalVector3d) */ {
 public:
  LocalVector3d();
  virtual ~LocalVector3d();

  LocalVector3d(const LocalVector3d& from);

  inline LocalVector3d& operator=(const LocalVector3d& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const LocalVector3d& default_instance();

  void Swap(LocalVector3d* other);

  // implements Message ----------------------------------------------

  inline LocalVector3d* New() const { return New(NULL); }

  LocalVector3d* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const LocalVector3d& from);
  void MergeFrom(const LocalVector3d& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const {
    return InternalSerializeWithCachedSizesToArray(false, output);
  }
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(LocalVector3d* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required double x = 1;
  bool has_x() const;
  void clear_x();
  static const int kXFieldNumber = 1;
  double x() const;
  void set_x(double value);

  // required double y = 2;
  bool has_y() const;
  void clear_y();
  static const int kYFieldNumber = 2;
  double y() const;
  void set_y(double value);

  // required double z = 3;
  bool has_z() const;
  void clear_z();
  static const int kZFieldNumber = 3;
  double z() const;
  void set_z(double value);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.LocalVector3d)
 private:
  inline void set_has_x();
  inline void clear_has_x();
  inline void set_has_y();
  inline void clear_has_y();
  inline void set_has_z();
  inline void clear_has_z();

  // helper for ByteSize()
  int RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  double x_;
  double y_;
  double z_;
  friend void  protobuf_AddDesc_local_5fvector3d_2eproto();
  friend void protobuf_AssignDesc_local_5fvector3d_2eproto();
  friend void protobuf_ShutdownFile_local_5fvector3d_2eproto();

  void InitAsDefaultInstance();
  static LocalVector3d* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// LocalVector3d

// required double x = 1;
inline bool LocalVector3d::has_x() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void LocalVector3d::set_has_x() {
  _has_bits_[0] |= 0x00000001u;
}
inline void LocalVector3d::clear_has_x() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void LocalVector3d::clear_x() {
  x_ = 0;
  clear_has_x();
}
inline double LocalVector3d::x() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.LocalVector3d.x)
  return x_;
}
inline void LocalVector3d::set_x(double value) {
  set_has_x();
  x_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.LocalVector3d.x)
}

// required double y = 2;
inline bool LocalVector3d::has_y() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void LocalVector3d::set_has_y() {
  _has_bits_[0] |= 0x00000002u;
}
inline void LocalVector3d::clear_has_y() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void LocalVector3d::clear_y() {
  y_ = 0;
  clear_has_y();
}
inline double LocalVector3d::y() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.LocalVector3d.y)
  return y_;
}
inline void LocalVector3d::set_y(double value) {
  set_has_y();
  y_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.LocalVector3d.y)
}

// required double z = 3;
inline bool LocalVector3d::has_z() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void LocalVector3d::set_has_z() {
  _has_bits_[0] |= 0x00000004u;
}
inline void LocalVector3d::clear_has_z() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void LocalVector3d::clear_z() {
  z_ = 0;
  clear_has_z();
}
inline double LocalVector3d::z() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.LocalVector3d.z)
  return z_;
}
inline void LocalVector3d::set_z(double value) {
  set_has_z();
  z_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.LocalVector3d.z)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_local_5fvector3d_2eproto__INCLUDED
