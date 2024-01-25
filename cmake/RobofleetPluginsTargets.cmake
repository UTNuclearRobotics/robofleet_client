include(CMakeFindDependencyMacro)

find_dependency(flatbuffers)

include("${CMAKE_CURRENT_LIST_DIR}/PluginsTargets.cmake")
include