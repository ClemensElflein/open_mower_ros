/*
 * OpenMower
 * Part of open_mower_ros (https://github.com/ClemensElflein/open_mower_ros)
 *
 * Copyright (C) 2026 The OpenMower Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <initializer_list>
#include <limits>

namespace utils {

/**
 * @brief Check if a sensor reading is valid
 *
 * Validates that a sensor reading is finite and above a minimum threshold.
 * This is useful for ROS float fields that default to 0.0 when unpublished,
 * but may get NaN once data arrives.
 *
 * @param value The sensor reading to validate
 * @param min_valid Minimum valid value (default 0.0f)
 * @return true if the reading is valid (finite and > min_valid)
 */
inline bool IsValidReading(float value, float min_valid = 0.0f) {
  return std::isfinite(value) && value > min_valid;
}

/**
 * @brief Get the first valid reading from provided values
 *
 * Returns the first value that passes isValidReading() check.
 * Useful for selecting between multiple sensor sources (e.g., ADC, CHG, BMS).
 *
 * @param values List of values to check in priority order
 * @param min_valid Minimum valid value (default 0.0f)
 * @param default_value Value to return if no valid reading is found (defaults to 0.0f)
 * @return The first valid value, or default_value if none are valid
 */
inline float GetFirstValid(std::initializer_list<float> values, float min_valid = 0.0f, float default_value = 0.0f) {
  for (float v : values) {
    if (IsValidReading(v, min_valid)) return v;
  }
  return default_value;
}

}  // namespace utils

#endif
