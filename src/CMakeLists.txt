target_sources(${PLUGINS_LIBRARY}
PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/ROSMsgHandlers.cpp
  ${CMAKE_CURRENT_LIST_DIR}/ROSSrvHandlers.cpp
  ${CMAKE_CURRENT_LIST_DIR}/MessageScheduler.cpp
  ${CMAKE_CURRENT_LIST_DIR}/common_conversions.cpp
PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/MessageSchedulerLib.hpp
)

target_sources(client
PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/main.cpp
  ${CMAKE_CURRENT_LIST_DIR}/WsClient.hpp
  ${CMAKE_CURRENT_LIST_DIR}/WsServer.hpp
  ${CMAKE_CURRENT_LIST_DIR}/RosClientNode.cpp
  ${CMAKE_CURRENT_LIST_DIR}/MessageScheduler.hpp
)