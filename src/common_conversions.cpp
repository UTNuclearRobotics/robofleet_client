#include <robofleet_client/common_conversions.hpp>


/********************************************************************
 * Function definitions for single primitive types.
 *******************************************************************/
std::string FbtoRos(const flatbuffers::String* src)
{
  return src->str();
}

flatbuffers::Offset<flatbuffers::String> RostoFb(flatbuffers::FlatBufferBuilder& fbb, const std::string& src)
{
  return fbb.CreateString(src.c_str());
}

std::u16string FbtoRos(const flatbuffers::Vector<char16_t>* src)
{
  return std::u16string(src->begin(), src->end());
}

flatbuffers::Offset<flatbuffers::Vector<char16_t>> RostoFb(flatbuffers::FlatBufferBuilder& fbb, const std::u16string& src)
{
  return fbb.CreateVector(std::vector<char16_t>(src.begin(), src.end()));
}