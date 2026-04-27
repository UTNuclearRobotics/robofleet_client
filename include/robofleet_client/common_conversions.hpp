#pragma once

#include <robofleet_client/base_schema_generated.h>
#include <rosidl_runtime_cpp/bounded_vector.hpp>

/********************************************************************
 * A set of functions and function templates needed to translate
 * data between ROS types and flatbuffers types.
 *******************************************************************/



/********************************************************************
 * Function templates for single primitive types. Most of these can be
 * simply passed between ROS and flatbuffers. Exceptions include time,
 * duration, and strings, so they get dedicated functions.
 *******************************************************************/

// translation between std::string and the flatbuffers string primitive 
std::string FbtoRos(const flatbuffers::String* src);

flatbuffers::Offset<flatbuffers::String> RostoFb(flatbuffers::FlatBufferBuilder& fbb, const std::string& src);


// translation between std::u16string and the flatbuffers ushort-vector

std::u16string FbtoRos(const flatbuffers::Vector<char16_t>* src);

flatbuffers::Offset<flatbuffers::Vector<char16_t>> RostoFb(flatbuffers::FlatBufferBuilder& fbb, const std::u16string& src);

/********************************************************************
 * Function templates for vectors of primitive types.
 *******************************************************************/

// for primitive types that don't require encoding/decoding
template<class PrimType>
std::vector<PrimType> FbtoRosPrimitive(const flatbuffers::Vector<PrimType>* src)
{
  return std::vector<PrimType>(src->begin(), src->end());
}

template<class PrimType>
flatbuffers::Offset<flatbuffers::Vector<PrimType>> RostoFbPrimitive(flatbuffers::FlatBufferBuilder& fbb, const std::vector<PrimType>& src)
{
  return fbb.CreateVector(src);
}

// for compound types that do require encoding/decoding
template<class RosType, class FbType>
std::vector<RosType> FbtoRos(const flatbuffers::Vector<flatbuffers::Offset<FbType>>* src)
{
  std::vector<RosType> dst;
  dst.reserve(src->size());
  typedef flatbuffers::Vector<flatbuffers::Offset<FbType>> fvec;
  for (typename fvec::const_iterator it = src->begin(); it != src->end(); ++it)
  {
    dst.push_back(FbtoRos(*it));
  }

  return dst;
}

template<class RosType, class FbType>
flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<FbType>>> RostoFb(flatbuffers::FlatBufferBuilder& fbb, const std::vector<RosType>& src)
{
  std::vector<flatbuffers::Offset<FbType>> dst(src.size());
  std::transform(
      src.begin(), src.end(), dst.begin(), [&fbb](const RosType& item) {
        return RostoFb(fbb, item);
      });
  return fbb.CreateVector(dst);
}



/********************************************************************
 * Function templates for arrays of primitive types.
 * ROS2 uses std::array to represent these.
 *******************************************************************/

// for primitive types that don't require encoding/decoding
template<class PrimType, size_t N>
std::array<PrimType, N> FbtoRosPrimitive(const flatbuffers::Vector<PrimType>* src)
{
  std::array<PrimType, N> output;
  std::copy_n(src->begin(), N, output.begin());

  return output;
}

template<class PrimType, size_t N>
flatbuffers::Offset<flatbuffers::Vector<PrimType>> RostoFbPrimitive(flatbuffers::FlatBufferBuilder& fbb, const std::array<PrimType, N>& src)
{
  std::vector<PrimType> temp;
  temp.resize(N);
  std::copy(src.begin(), src.end(), temp.begin());
  return fbb.CreateVector(temp);
}

// for compound types that do require encoding/decoding
template<class RosType, class FbType, size_t N>
std::array<RosType, N> FbtoRos(const flatbuffers::Vector<FbType>* src)
{
  // protection against writing past the end of thee output array
  typename flatbuffers::Vector<FbType>::const_iterator end_it = src->end();
  if (src->size() > N)
  {
    end_it = std::advance(src->begin(), N);
  }

  std::array<RosType, N> output;
  std::transform(
      src->begin(), end_it, output.begin(), [](const RosType& item) {
        return FbtoRos(item);
      });

  return output;
}

template<class RosType, class FbType, size_t N>
flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<FbType>>>
RostoFb(flatbuffers::FlatBufferBuilder& fbb, const std::array<RosType, N>& src)
{
  std::vector<flatbuffers::Offset<FbType>> dst(src.size());
  std::transform(
      src.begin(), src.end(), dst.begin(), [&fbb](const RosType& item) {
        return RostoFb(fbb, item);
      });
  return fbb.CreateVector(dst);
}

// To support vector of strings
template<>
inline std::vector<std::string> FbtoRos<std::string, flatbuffers::String>(
    const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>* src)
{
  std::vector<std::string> dst;
  if (src) {
    dst.reserve(src->size());
    for (auto it = src->begin(); it != src->end(); ++it) {
      dst.push_back(it->str());  // Convert from flatbuffers::String to std::string
    }
  }
  return dst;
}

// Special case for std::vector<std::string> → FlatBuffers vector of strings
template<>
inline flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>>
RostoFb<std::string, flatbuffers::String>(flatbuffers::FlatBufferBuilder& fbb, const std::vector<std::string>& src)
{
  std::vector<flatbuffers::Offset<flatbuffers::String>> dst;
  dst.reserve(src.size());
  for (const auto& s : src) {
    dst.push_back(RostoFb(fbb, s));
  }
  return fbb.CreateVector(dst);
}

// In flatBuffers, there's no native bool[] type
// then we need funtions to support bool arrays.
inline std::vector<bool> FbtoRosBoolArray(const flatbuffers::Vector<uint8_t>* src)
{
  std::vector<bool> output;
  output.reserve(src->size());
  for (auto v : *src) {
    output.push_back(static_cast<bool>(v));
  }
  return output;
}

inline flatbuffers::Offset<flatbuffers::Vector<uint8_t>> RostoFbBoolArray(flatbuffers::FlatBufferBuilder& fbb, const std::vector<bool>& src)
{
  std::vector<uint8_t> temp;
  temp.reserve(src.size());
  for (bool b : src) {
    temp.push_back(static_cast<uint8_t>(b));
  }
  return fbb.CreateVector(temp);
}


// Bounded Vectors

template<class RosType, class FbType>
RosType ConvertElement(const FbType* src);

template<class RosType, class FbType>
flatbuffers::Offset<FbType>
ConvertElementToFb(flatbuffers::FlatBufferBuilder& fbb, const RosType& src);


template<class RosType, class FbType, size_t MaxN>
rosidl_runtime_cpp::BoundedVector<RosType, MaxN>
FbtoRosBounded(const flatbuffers::Vector<flatbuffers::Offset<FbType>>* src)
{
  rosidl_runtime_cpp::BoundedVector<RosType, MaxN> dst;

  const size_t count = std::min<size_t>(src->size(), MaxN);
  dst.reserve(count);

  for (size_t i = 0; i < count; i++) {
    dst.push_back(ConvertElement<RosType, FbType>(src->Get(i)));
  }

  return dst;
}

template<class RosType, class FbType, size_t MaxN>
flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<FbType>>>
RostoFbBounded(flatbuffers::FlatBufferBuilder& fbb,
               const rosidl_runtime_cpp::BoundedVector<RosType, MaxN>& src)
{
  std::vector<flatbuffers::Offset<FbType>> temp;
  temp.reserve(src.size());

  for (const auto& item : src) {
    temp.push_back(ConvertElementToFb<RosType, FbType>(fbb, item));
  }

  return fbb.CreateVector(temp);
}

// Bounded Vectors - Primitives
template<class PrimType, size_t MaxN>
rosidl_runtime_cpp::BoundedVector<PrimType, MaxN>
FbtoRosPrimitiveBounded(const flatbuffers::Vector<PrimType>* src);

template<class PrimType, size_t MaxN>
flatbuffers::Offset<flatbuffers::Vector<PrimType>>
RostoFbPrimitiveBounded(flatbuffers::FlatBufferBuilder& fbb,
                        const rosidl_runtime_cpp::BoundedVector<PrimType, MaxN>& src);