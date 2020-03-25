#define BUZZER_PIN 6

// // C major
// #define NOTE_C0 -1
// #define NOTE_C1 262
// #define NOTE_C2 294
// #define NOTE_C3 330
// #define NOTE_C4 350
// #define NOTE_C5 393
// #define NOTE_C6 441
// #define NOTE_C7 495
// // D major
// #define NOTE_D0 -1
// #define NOTE_D1 294
// #define NOTE_D2 330
// #define NOTE_D3 350
// #define NOTE_D4 393
// #define NOTE_D5 441
// #define NOTE_D6 495
// #define NOTE_D7 556

// #define NOTE_DL1 147
// #define NOTE_DL2 165
// #define NOTE_DL3 175
// #define NOTE_DL4 196
// #define NOTE_DL5 221
// #define NOTE_DL6 248
// #define NOTE_DL7 278

// #define NOTE_DH1 589
// #define NOTE_DH2 661
// #define NOTE_DH3 700
// #define NOTE_DH4 786
// #define NOTE_DH5 882
// #define NOTE_DH6 990
// #define NOTE_DH7 112

// // G major
// #define NOTE_G0 -1
// #define NOTE_G1 393
// #define NOTE_G2 441
// #define NOTE_G3 495
// #define NOTE_G4 556
// #define NOTE_G5 624
// #define NOTE_G6 661
// #define NOTE_G7 742

// #define NOTE_GL1 196
// #define NOTE_GL2 221
// #define NOTE_GL3 234
// #define NOTE_GL4 262
// #define NOTE_GL5 294
// #define NOTE_GL6 330
// #define NOTE_GL7 371

// #define NOTE_GH1 786
// #define NOTE_GH2 882
// #define NOTE_GH3 990
// #define NOTE_GH4 1049
// #define NOTE_GH5 1178
// #define NOTE_GH6 1322
// #define NOTE_GH7 1484

// int tune[] =
//     {
//         NOTE_C1, NOTE_C5, NOTE_C0, NOTE_C6, NOTE_C3};

// float duration[] =
//     {
//         0.8, 0.5, 0.3, 0.6 + 0.3, 0.3};

// /*
// * Beep once using delay
// */

// void beep_once()
// {
//   pinMode(BUZZER_PIN, OUTPUT);
//   analogWrite(BUZZER_PIN, 100);
//   delay(1000);
//   analogWrite(BUZZER_PIN, 0);
// }

// /*
// * Play the tone of SNCF ring.
// * Using delay.
// */

// void play_tone()
// {
//   int length = sizeof(tune) / sizeof(tune[0]);
//   for (int x = 0; x < length; x++)
//   {
//     tone(BUZZER_PIN, tune[x]);
//     delay(400 * duration[x]);
//     noTone(BUZZER_PIN);
//   }
// }
