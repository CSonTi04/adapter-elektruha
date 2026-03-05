// tests/test_logic.cpp
// Host-side unit tests for the pure-logic helpers in src/logic.h.
//
// Build and run (doctest.h must be present in tests/):
//   g++ -std=c++17 -Wall -I.. -o /tmp/test_runner tests/test_logic.cpp
//   /tmp/test_runner
//
// See README.md § "CI / Testing" for one-liner download of doctest.h.

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "../src/logic.h"

// =============================================================================
// clamp01
// =============================================================================

TEST_CASE("clamp01 passes through values already in [0, 1]") {
  CHECK(clamp01(0.0f)  == doctest::Approx(0.0f));
  CHECK(clamp01(0.5f)  == doctest::Approx(0.5f));
  CHECK(clamp01(1.0f)  == doctest::Approx(1.0f));
}

TEST_CASE("clamp01 clamps values below 0") {
  CHECK(clamp01(-0.01f)  == doctest::Approx(0.0f));
  CHECK(clamp01(-1.0f)   == doctest::Approx(0.0f));
  CHECK(clamp01(-100.0f) == doctest::Approx(0.0f));
}

TEST_CASE("clamp01 clamps values above 1") {
  CHECK(clamp01(1.01f)  == doctest::Approx(1.0f));
  CHECK(clamp01(2.0f)   == doctest::Approx(1.0f));
  CHECK(clamp01(100.0f) == doctest::Approx(1.0f));
}

// =============================================================================
// chooseState — priority ordering
// =============================================================================

TEST_CASE("chooseState returns Idle when all emotions are at rest") {
  Emotion E;
  // lastTouchMs = 0, nowMs = 5000 -> 5000ms since touch > 1500ms window
  CHECK(chooseState(E, 5000) == MoodState::Idle);
}

TEST_CASE("chooseState returns Friendly within the 1500 ms touch window") {
  Emotion E;
  E.lastTouchMs = 4000;
  CHECK(chooseState(E, 4001) == MoodState::Friendly); // 1 ms in
  CHECK(chooseState(E, 5499) == MoodState::Friendly); // 1499 ms in
}

TEST_CASE("chooseState touch window expires at exactly 1500 ms") {
  Emotion E;
  E.lastTouchMs = 4000;
  // 4000 + 1500 = 5500; 5500 - 4000 = 1500, NOT < 1500
  CHECK(chooseState(E, 5500) != MoodState::Friendly);
  // 5499 - 4000 = 1499 < 1500 → still Friendly
  CHECK(chooseState(E, 5499) == MoodState::Friendly);
}

TEST_CASE("chooseState returns Anxious when anxiety > 0.65") {
  Emotion E;
  E.lastTouchMs = 0;
  E.anxiety = 0.70f;
  CHECK(chooseState(E, 5000) == MoodState::Anxious);
}

TEST_CASE("chooseState Anxious threshold boundary: 0.65 is NOT sufficient") {
  Emotion E;
  E.lastTouchMs = 0;
  E.anxiety = 0.65f;
  CHECK(chooseState(E, 5000) == MoodState::Idle); // 0.65 is not > 0.65

  E.anxiety = 0.651f;
  CHECK(chooseState(E, 5000) == MoodState::Anxious);
}

TEST_CASE("chooseState returns Excited when arousal > 0.55") {
  Emotion E;
  E.lastTouchMs = 0;
  E.arousal = 0.60f;
  CHECK(chooseState(E, 5000) == MoodState::Excited);
}

TEST_CASE("chooseState Excited threshold boundary: 0.55 is NOT sufficient") {
  Emotion E;
  E.lastTouchMs = 0;
  E.arousal = 0.55f;
  CHECK(chooseState(E, 5000) == MoodState::Idle); // 0.55 is not > 0.55

  E.arousal = 0.551f;
  CHECK(chooseState(E, 5000) == MoodState::Excited);
}

TEST_CASE("chooseState priority: Friendly beats Anxious and Excited") {
  Emotion E;
  E.lastTouchMs  = 4800;
  E.anxiety      = 0.90f;
  E.arousal      = 0.90f;
  // 5000 - 4800 = 200 ms < 1500 → Friendly wins
  CHECK(chooseState(E, 5000) == MoodState::Friendly);
}

TEST_CASE("chooseState priority: Anxious beats Excited") {
  Emotion E;
  E.lastTouchMs = 0;   // touch window expired
  E.anxiety     = 0.90f;
  E.arousal     = 0.90f;
  CHECK(chooseState(E, 5000) == MoodState::Anxious);
}

// =============================================================================
// computeMaskBinary — thresholding and gamma shaping
// =============================================================================

TEST_CASE("computeMaskBinary returns 0x00 when all targets are zero") {
  float targets[8] = {0.0f};
  CHECK(computeMaskBinary(targets, 0.22f) == 0x00);
}

TEST_CASE("computeMaskBinary returns 0xFF when all targets are 1.0") {
  float targets[8] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
  CHECK(computeMaskBinary(targets, 0.22f) == 0xFF);
}

TEST_CASE("computeMaskBinary gamma shaping: threshold applies to target^2") {
  // threshold = 0.22; channel ON iff target^2 >= 0.22, i.e. target >= ~0.469
  // 0.47^2 = 0.2209 >= 0.22 → ON
  // 0.46^2 = 0.2116 <  0.22 → OFF
  float targets[8] = {0.0f};
  targets[0] = 0.47f; // ON
  targets[1] = 0.46f; // OFF
  uint8_t mask = computeMaskBinary(targets, 0.22f);
  CHECK((mask & (1 << 0)) != 0); // ch0 ON
  CHECK((mask & (1 << 1)) == 0); // ch1 OFF
}

TEST_CASE("computeMaskBinary sets correct independent bits") {
  float targets[8] = {0.0f};
  targets[2] = 0.50f; // 0.25 >= 0.22 → ON  → bit 2
  targets[5] = 0.40f; // 0.16 <  0.22 → OFF → bit 5 clear
  targets[7] = 0.80f; // 0.64 >= 0.22 → ON  → bit 7
  uint8_t mask = computeMaskBinary(targets, 0.22f);
  CHECK((mask & (1 << 2)) != 0); // ch2 ON
  CHECK((mask & (1 << 5)) == 0); // ch5 OFF
  CHECK((mask & (1 << 7)) != 0); // ch7 ON
}

TEST_CASE("computeMaskBinary respects custom threshold") {
  // With a high threshold (0.90) only targets near 1.0 pass
  float targets[8] = {0.0f};
  targets[3] = 0.95f; // 0.9025 >= 0.90 → ON
  targets[4] = 0.94f; // 0.8836 <  0.90 → OFF
  uint8_t mask = computeMaskBinary(targets, 0.90f);
  CHECK((mask & (1 << 3)) != 0); // ch3 ON
  CHECK((mask & (1 << 4)) == 0); // ch4 OFF
}
