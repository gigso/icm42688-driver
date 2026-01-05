zephyr_library()
zephyr_library_sources(icm42688.c)

zephyr_library_sources_ifdef(CONFIG_ICM42688_FIFO    icm42688_fifo.c icm42688_fifo_queue.c)
zephyr_library_sources_ifdef(CONFIG_ICM42688_TRIGGER icm42688_trigger.c)
zephyr_library_sources_ifdef(CONFIG_ICM42688_APEX    icm42688_apex.c)

# Advanced filter modules (always compiled)
zephyr_library_sources(icm42688_notch.c)
zephyr_library_sources(icm42688_aaf.c)

# Export public headers so app can do: #include "icm42688.h"
# Adjust paths to where your header actually lives.
zephyr_library_include_directories(
  .
  ${CMAKE_CURRENT_LIST_DIR}/../include
  ${CMAKE_CURRENT_LIST_DIR}/../../include
)
