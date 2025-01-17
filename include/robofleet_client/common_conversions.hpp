#pragma once

#include <robofleet_client/base_schema_generated.h>

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
  return fbb.CreateVector(dst).o;
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
flatbuffers::Offset<flatbuffers::Vector<FbType>> RostoFb(flatbuffers::FlatBufferBuilder& fbb, const std::array<RosType, N>& src)
{
  std::vector<flatbuffers::Offset<FbType>> dst(src.size());
  std::transform(
      src.begin(), src.end(), dst.begin(), [&fbb](const RosType& item) {
        return RostoFb(fbb, item);
      });
  return fbb.CreateVector(dst).o;
}