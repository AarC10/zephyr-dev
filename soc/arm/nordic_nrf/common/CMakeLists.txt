# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_SOC_FAMILY_NRF soc_nrf_common.S)
zephyr_library_sources_ifdef(CONFIG_POWEROFF poweroff.c)
zephyr_include_directories(.)

if (CONFIG_TFM_PARTITION_PLATFORM)
  zephyr_sources(soc_secure.c)
  zephyr_library_include_directories(
    $<TARGET_PROPERTY:tfm,TFM_BINARY_DIR>/api_ns/interface/include
  )
endif()
