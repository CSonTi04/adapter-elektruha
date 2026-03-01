// =============================================================================
// 00_language_concepts – Language Tour for Absolute Beginners
// =============================================================================
// This sketch is NOT part of the main project logic.
// It is a guided tour of every programming concept used in this curriculum,
// written for people who have never programmed before.
//
// Upload it, open Serial Monitor at 115200 baud, and read along with the
// comments below.  Nothing breaks if you experiment — you can always
// re-upload the original.
//
// KEY IDEA: A program is a list of instructions the computer follows in order,
// from top to bottom, unless you tell it to repeat or skip ahead.
// =============================================================================

// =============================================================================
// SECTION 1 – COMMENTS
// =============================================================================
// Everything after "//" on a line is a COMMENT.
// The computer ignores it completely — it is there only for humans to read.
// Use comments to explain WHY you wrote something, not just what it does.
//
// Multi-line comments start with /* and end with */:
/* This whole block is a comment too.
   Useful when you want to "comment out" a big chunk of code temporarily. */


// =============================================================================
// SECTION 2 – VARIABLES  (named boxes that hold values)
// =============================================================================
// A variable is like a labelled sticky note: it has a name and a value.
// You must say what TYPE of value it holds before you can use it.
//
// Common types:
//   int       – a whole number, e.g. -7, 0, 42  (range: roughly ±2 billion)
//   uint32_t  – a POSITIVE-only whole number, 0 to 4,294,967,295
//               (used for millisecond timestamps because they only count up)
//   float     – a decimal number, e.g. 3.14, -0.5, 1000.0
//   bool      – only two possible values: true or false
//   char      – a single letter/character, e.g. 'A', '3', '!'

int    myInteger  = 42;        // whole number, starts at 42
float  myDecimal  = 3.14f;     // decimal; the "f" means "float constant"
bool   myFlag     = false;     // starts as false (not true)
uint32_t counter  = 0;         // starts at zero, counts up

// You can change a variable any time:
//   myInteger = 100;
// But you cannot change its TYPE after declaring it.


// =============================================================================
// SECTION 3 – CONSTANTS  (variables that must never change)
// =============================================================================
// Sometimes a value should NEVER change during the program.
// Two ways to create a constant:

//   constexpr – modern C++ way (preferred in this curriculum)
constexpr int    LED_PIN  = 2;      // GPIO2 is the built-in LED on most boards
constexpr float  PI_APPROX = 3.14159f;

//   #define – old-school preprocessor constant (text substitution before compile)
//   The compiler literally replaces every "MY_DELAY" with 500 in the code.
#define MY_DELAY 500

// Why bother with constants instead of just writing the number?
//   1) It gives the number a meaningful name (LED_PIN is clearer than 2).
//   2) You only need to change one line if the value ever needs updating.


// =============================================================================
// SECTION 4 – ARITHMETIC AND COMPARISON OPERATORS
// =============================================================================
// Arithmetic:
//   +   add
//   -   subtract
//   *   multiply
//   /   divide (whole-number division if both sides are int: 7/2 = 3, not 3.5!)
//   %   modulo – the REMAINDER after division: 7 % 3 = 1, 10 % 5 = 0
//
// Comparison (result is always true or false):
//   ==  equal to          (note: TWO equals signs, not one!)
//   !=  not equal to
//   >   greater than
//   <   less than
//   >=  greater than or equal to
//   <=  less than or equal to
//
// Logic (combining multiple conditions):
//   &&  AND – both conditions must be true
//   ||  OR  – at least one condition must be true
//   !   NOT – flips true→false or false→true

// No runnable example here; see the if-statement section below.


// =============================================================================
// SECTION 5 – if / else  (making decisions)
// =============================================================================
// An "if" block runs its code ONLY when the condition in ( ) is true.
// An optional "else" block runs when the condition is false.
//
// Format:
//   if (CONDITION) {
//     // runs when CONDITION is true
//   } else {
//     // runs when CONDITION is false  (optional)
//   }

void demonstrateIf() {
  int temperature = 22;

  if (temperature > 30) {
    Serial.println("It is hot!");
  } else if (temperature > 15) {
    Serial.println("It is comfortable.");   // ← this one runs
  } else {
    Serial.println("It is cold!");
  }
  // "else if" lets you chain multiple checks in order.
}


// =============================================================================
// SECTION 6 – for LOOP  (repeat a fixed number of times)
// =============================================================================
// A "for" loop repeats a block of code a set number of times.
//
// Format:
//   for (START; CONDITION; STEP) {
//     // code to repeat
//   }
//
//   START     – run once before the loop begins (usually "int i = 0")
//   CONDITION – checked BEFORE each iteration; stop when false
//   STEP      – run after each iteration (usually "i++" meaning add 1)

void demonstrateForLoop() {
  // Count from 0 to 4 (five iterations):
  for (int i = 0; i < 5; i++) {
    Serial.print("  Step ");
    Serial.println(i);   // prints 0, 1, 2, 3, 4
  }

  // Count DOWN from 7 to 0:
  for (int i = 7; i >= 0; i--) {
    Serial.print("  Countdown ");
    Serial.println(i);
  }
}


// =============================================================================
// SECTION 7 – while LOOP  (repeat as long as something is true)
// =============================================================================
// A "while" loop keeps repeating as long as its condition stays true.
// Unlike a "for" loop, you manage the counter yourself.
//
// WARNING: If the condition is ALWAYS true, the loop never ends!
//          That is called an "infinite loop" and will freeze your program.
//          (In Arduino, loop() itself is an intentional infinite loop.)

void demonstrateWhileLoop() {
  int count = 0;
  while (count < 3) {
    Serial.print("  while count=");
    Serial.println(count);
    count++;   // "++" means "add 1" — same as count = count + 1
  }
  // After this, count == 3, so the condition (count < 3) is false → loop ends.
}


// =============================================================================
// SECTION 8 – FUNCTIONS  (reusable named blocks of code)
// =============================================================================
// A function is a named block of code you can "call" (run) from anywhere.
// Functions help avoid copy-pasting the same code repeatedly.
//
// Format:
//   RETURN_TYPE functionName(PARAMETER_TYPE paramName, ...) {
//     // code
//     return SOME_VALUE;   // only needed if RETURN_TYPE is not "void"
//   }
//
// RETURN_TYPE:
//   void  – the function does something but gives nothing back
//   int   – the function gives back a whole number
//   float – the function gives back a decimal number
//   bool  – the function gives back true or false
//   etc.
//
// PARAMETERS are values you pass IN to the function (like ingredients to a recipe).

// A function that takes two numbers and returns their average:
float average(float a, float b) {
  return (a + b) / 2.0f;
}

// A void function that just does something (no return value):
void printSeparator() {
  Serial.println("----------------------------");
}


// =============================================================================
// SECTION 9 – ARRAYS  (a row of variables with one name)
// =============================================================================
// An array is a numbered list of values of the same type.
// Items are numbered starting from 0 (not 1!).
//
// Format:
//   TYPE name[SIZE];
//   TYPE name[SIZE] = { value0, value1, value2, ... };
//
// Access an item by its position (index):
//   name[0]   ← first item
//   name[1]   ← second item
//   name[SIZE-1] ← last item

void demonstrateArrays() {
  int readings[4] = {10, 20, 30, 40};

  Serial.print("  First item:  "); Serial.println(readings[0]);   // 10
  Serial.print("  Second item: "); Serial.println(readings[1]);   // 20
  Serial.print("  Last item:   "); Serial.println(readings[3]);   // 40

  // Loop over every item:
  for (int i = 0; i < 4; i++) {
    Serial.print("  readings["); Serial.print(i);
    Serial.print("] = "); Serial.println(readings[i]);
  }
}


// =============================================================================
// SECTION 10 – THE TERNARY OPERATOR  (compact if/else in one line)
// =============================================================================
// Format:   CONDITION ? VALUE_IF_TRUE : VALUE_IF_FALSE
// Handy for choosing between two values without a full if/else block.

void demonstrateTernary() {
  int x = 7;
  const char* result = (x > 5) ? "big" : "small";
  // If x > 5 is true, result = "big".  Otherwise result = "small".
  Serial.print("  x is: "); Serial.println(result);   // "big"
}


// =============================================================================
// SECTION 11 – struct  (grouping related variables together)
// =============================================================================
// A "struct" (structure) bundles several variables under one name.
// Instead of having separate variables arousal, anxiety, affection,
// you can package them into one neat container.
//
// Think of it like a form with labelled fields.

struct EmotionState {
  float arousal;    // 0.0 = calm, 1.0 = very aroused
  float anxiety;    // 0.0 = relaxed, 1.0 = very anxious
  float affection;  // 0.0 = neutral, 1.0 = very affectionate
};

// Create one instance of the struct:
EmotionState emotions = { 0.2f, 0.1f, 0.5f };  // arousal=0.2, anxiety=0.1, affection=0.5

void demonstrateStruct() {
  // Access fields with a dot:
  Serial.print("  arousal=");   Serial.println(emotions.arousal);
  Serial.print("  anxiety=");   Serial.println(emotions.anxiety);

  // Modify a field:
  emotions.arousal = 0.8f;
  Serial.print("  arousal after boost="); Serial.println(emotions.arousal);
}


// =============================================================================
// SECTION 12 – enum class  (a set of named choices)
// =============================================================================
// An "enum" (enumeration) gives names to a fixed set of options.
// Instead of using magic numbers (0=Idle, 1=Excited, 2=Anxious, 3=Friendly),
// you write the names directly.  This makes the code much easier to read.
//
// "class" after "enum" means the names stay private to this enum — you write
// MoodState::Idle instead of just Idle.

enum class MoodState : uint8_t {   // uint8_t means it fits in 1 byte
  Idle,
  Excited,
  Anxious,
  Friendly
};

void demonstrateEnum() {
  MoodState current = MoodState::Idle;

  // switch/case: like a series of if/else but cleaner for enum comparisons
  switch (current) {
    case MoodState::Idle:     Serial.println("  State: Idle");     break;
    case MoodState::Excited:  Serial.println("  State: Excited");  break;
    case MoodState::Anxious:  Serial.println("  State: Anxious");  break;
    case MoodState::Friendly: Serial.println("  State: Friendly"); break;
    // "break" is IMPORTANT — without it the code "falls through" to the next case!
  }
}


// =============================================================================
// SECTION 13 – #include  (importing a library)
// =============================================================================
// A library is a bundle of pre-written functions you can use in your sketch.
// "#include <LibraryName.h>" makes those functions available.
// Examples used in this curriculum:
//   #include <Wire.h>       – I2C communication
//   #include <PCF8574.h>    – PCF8574 expander control
//   #include <BLEDevice.h>  – Bluetooth LE scanning
//
// The angle brackets < > mean the file is in the standard library path.
// Double quotes "MyFile.h" mean the file is in the same folder as your sketch.
//
// (No demo here — you need hardware for most library features.)


// =============================================================================
// SECTION 14 – setup() and loop()  (the Arduino model)
// =============================================================================
// Every Arduino sketch has exactly two special functions:
//
//   setup()  – runs ONCE when the board powers on or resets
//              Use it for one-time initialisation (start Serial, set pin modes…)
//
//   loop()   – runs OVER AND OVER FOREVER after setup() finishes
//              This is where your main program logic lives.
//
// The Arduino framework calls these for you — you just fill them in.


// =============================================================================
// SECTION 15 – millis()  (measuring time without blocking)
// =============================================================================
// millis() returns the number of milliseconds since the board was powered on.
// (1000 ms = 1 second)
//
// The key trick for non-blocking timing:
//   Remember WHEN you last did something (lastMs).
//   Each time through loop(), check if enough time has passed.
//   If yes, do the action and update lastMs.
//   If no, do nothing and return — loop() will check again very soon.
//
// This is MUCH better than delay(500), which freezes the whole board for 500 ms
// and prevents it from doing ANYTHING else (reading sensors, updating LEDs, etc.).
//
// See sketch 01_heartbeat_millis for a live demonstration.


// =============================================================================
// MAIN PROGRAM
// =============================================================================

uint32_t lastDemoMs = 0;
constexpr uint32_t DEMO_INTERVAL_MS = 4000;  // run a new demo section every 4 s
int demoSection = 0;

void setup() {
  Serial.begin(115200);
  delay(500);  // small pause so Serial Monitor connects before first output

  Serial.println();
  Serial.println("==========================================================");
  Serial.println(" 00_language_concepts – Language Tour for Beginners");
  Serial.println("==========================================================");
  Serial.println("Read the comments in this file alongside the Serial output.");
  Serial.println("A new demo runs every 4 seconds.  Reset the board to repeat.");
  Serial.println();

  Serial.println("--- Section 5: if / else ---");
  demonstrateIf();
  Serial.println();
}

void loop() {
  uint32_t now = millis();

  // Only run the next demo section every DEMO_INTERVAL_MS milliseconds.
  // This is the non-blocking timing pattern from sketch 01!
  if (now - lastDemoMs < DEMO_INTERVAL_MS) return;
  lastDemoMs = now;

  demoSection++;

  switch (demoSection) {
    case 1:
      Serial.println("--- Section 6: for loop ---");
      demonstrateForLoop();
      break;

    case 2:
      Serial.println("--- Section 7: while loop ---");
      demonstrateWhileLoop();
      break;

    case 3:
      Serial.println("--- Section 8: functions ---");
      Serial.print("  average(3.0, 7.0) = ");
      Serial.println(average(3.0f, 7.0f));
      printSeparator();
      break;

    case 4:
      Serial.println("--- Section 9: arrays ---");
      demonstrateArrays();
      break;

    case 5:
      Serial.println("--- Section 10: ternary operator ---");
      demonstrateTernary();
      break;

    case 6:
      Serial.println("--- Section 11: struct ---");
      demonstrateStruct();
      break;

    case 7:
      Serial.println("--- Section 12: enum class + switch/case ---");
      demonstrateEnum();
      break;

    case 8:
      Serial.println("--- Section 15: millis() demo ---");
      Serial.print("  millis() right now = ");
      Serial.print(now);
      Serial.println(" ms since power-on");
      Serial.println();
      Serial.println("All sections shown!  Reset the board to repeat.");
      Serial.println("Now open sketch 01_heartbeat_millis to see these");
      Serial.println("concepts used in a real project context.");
      break;

    default:
      // Nothing more to show — just sit quietly.
      break;
  }

  Serial.println();
}
