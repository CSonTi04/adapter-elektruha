// src/logic.h
// Pure-logic helpers extracted from anxius.ino.
//
// These implementations mirror the sketch exactly so that host-side unit tests
// can validate the business logic without Arduino or ESP32 headers.
// The original anxius.ino is unchanged and does not include this file.

#pragma once

#include <cstdint>

// ---- Mood state (mirrors anxius.ino enum) ----

enum class MoodState : uint8_t {
  Idle,
  Excited,
  Anxious,
  Friendly
};

// ---- Emotional model (mirrors anxius.ino struct) ----

struct Emotion {
  float    arousal         = 0.0f;
  float    anxiety         = 0.0f;
  float    affection       = 0.0f;
  uint32_t lastTouchMs     = 0;
  int      lastSeenDevices = 0;
};

// ---- clamp01 ----
// Clamp x to [0, 1].  Mirrors the free function in anxius.ino.

inline float clamp01(float x) {
  return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
}

// ---- chooseState ----
// Priority-based mood selector.  Mirrors chooseState() in anxius.ino exactly:
//   1. Friendly  — within 1500 ms of last touch (highest priority)
//   2. Anxious   — anxiety > 0.65
//   3. Excited   — arousal  > 0.55
//   4. Idle      — default baseline

inline MoodState chooseState(const Emotion& E, uint32_t nowMs) {
  if (nowMs - E.lastTouchMs < 1500u) return MoodState::Friendly;
  if (E.anxiety > 0.65f)             return MoodState::Anxious;
  if (E.arousal > 0.55f)             return MoodState::Excited;
  return MoodState::Idle;
}

// ---- computeMaskBinary ----
// Binary ON/OFF mask from 8-channel brightness targets.
// Applies the same quadratic (gamma) shaping used in the sketch, then
// compares against the supplied threshold.

inline uint8_t computeMaskBinary(const float ledTarget[8], float threshold) {
  uint8_t mask = 0;
  for (int i = 0; i < 8; i++) {
    float b = ledTarget[i] * ledTarget[i]; // same gamma shaping as sketch
    if (b >= threshold) mask |= static_cast<uint8_t>(1u << i);
  }
  return mask;
}
