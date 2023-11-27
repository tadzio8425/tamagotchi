#include "music.h"
#include "driver/ledc.h"

#include <stdio.h>
#include <math.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "music.h"

// change this to make the song slower or faster
int tempo = 108;


int song[] = {
  
  // Zelda's Lullaby - The Legend of Zelda Ocarina of Time. 
  // Score available at https://musescore.com/user/12754451/scores/2762776
  
  NOTE_E4,2, NOTE_G4,4,
  NOTE_D4,2, NOTE_C4,8, NOTE_D4,8, 
  NOTE_E4,2, NOTE_G4,4,
  NOTE_D4,-2,
  NOTE_E4,2, NOTE_G4,4,
  NOTE_D5,2, NOTE_C5,4,
  NOTE_G4,2, NOTE_F4,8, NOTE_E4,8, 
  NOTE_D4,-2,
  NOTE_E4,2, NOTE_G4,4,
  NOTE_D4,2, NOTE_C4,8, NOTE_D4,8, 
  NOTE_E4,2, NOTE_G4,4,
  NOTE_D4,-2,
  NOTE_E4,2, NOTE_G4,4,

  NOTE_D5,2, NOTE_C5,4,
  NOTE_G4,2, NOTE_F4,8, NOTE_E4,8, 
  NOTE_F4,8, NOTE_E4,8, NOTE_C4,2,
  NOTE_F4,2, NOTE_E4,8, NOTE_D4,8, 
  NOTE_E4,8, NOTE_D4,8, NOTE_A3,2,
  NOTE_G4,2, NOTE_F4,8, NOTE_E4,8, 
  NOTE_F4,8, NOTE_E4,8, NOTE_C4,4, NOTE_F4,4,
  NOTE_C5,-2, 
  
};

void tone(unsigned char dur_hms, uint32_t note)
{
	ledc_timer_config_t ledc_timer;
	ledc_channel_config_t ledc_channel;

	ledc_timer.duty_resolution = LEDC_TIMER_13_BIT;	// resolution of PWM duty
	ledc_timer.freq_hz = note;						// frequency of PWM signal
	ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;			// timer mode
	ledc_timer.timer_num = LEDC_TIMER_0;			// timer index

	// Set configuration of timer0 for high speed channels
	ledc_timer_config(&ledc_timer);
	
	ledc_channel.channel    = LEDC_CHANNEL_0;
	ledc_channel.duty       = 4096;
	ledc_channel.gpio_num   = BUZZER_GPIO;
	ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
	ledc_channel.hpoint     = 0;
	ledc_channel.timer_sel  = LEDC_TIMER_0;

	ledc_channel_config(&ledc_channel);
	vTaskDelay(pdMS_TO_TICKS(dur_hms*100));
	ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,0);
}



void playSong(){


    // sizeof gives the number of bytes, each int value is composed of two bytes (16 bits)
    // there are two values per note (pitch and duration), so for each note there are four bytes
    int notes = sizeof(song) / sizeof(song[0]) / 2;

    // this calculates the duration of a whole note in ms
    int wholenote = (60000 * 4) / tempo;

    int divider = 0, noteDuration = 0;

    // iterate over the notes of the melody. 
    // Remember, the array is twice the number of notes (notes + durations)
    for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {

    // calculates the duration of each note
    divider = song[thisNote + 1];
    if (divider > 0) {
    // regular note, just proceed
        noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
    // dotted notes are represented with negative durations!!
        noteDuration = (wholenote) / abs(divider);
        noteDuration *= 1.5; // increases the duration in half for dotted notes
    }

    // we only play the note for 90% of the duration, leaving 10% as a pause
    tone(noteDuration*0.9, song[thisNote]);
    }
}
