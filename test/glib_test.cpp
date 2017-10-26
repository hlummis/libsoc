#include <glib.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/wait.h>
#include <ctime>
#include "libsoc_gpio.h"
#include "libsoc_debug.h"

#define PING_OUTPUT 38
#define PING_INPUT 36
#define RESET_OUTPUT 63 // TODO: set this
#define PING_INTERVAL 10

typedef struct {
  gpio *ping_output;
  gpio *ping_input;
  gpio *reset_output;
  std::time_t prevPing;
  std::time_t curPing;
} pinData;

/**
 * Send a momentary ping signal
 */
static gboolean
sendPing (gpointer args)
{
  // Cast args to the correct pointer type
  pinData *pins;
  pins = (pinData *) args;

  // Send a short ping
  libsoc_gpio_set_level (pins->ping_output, HIGH);
  libsoc_gpio_set_level (pins->ping_output, LOW);

  return TRUE;
}

gboolean sendReset (void *args)
{
  // Cast args to the correct pointer type
  pinData *pins;
  pins = (pinData *) args;

  // Send a reset signal
  libsoc_gpio_set_level (pins->reset_output, HIGH);
  // Hold signal for a moment
  sleep (100);
  // Unset the reset signal
  libsoc_gpio_set_level (pins->reset_output, LOW);

  return TRUE;
}

// static gboolean
// resetTimeoutCallback () {}

/**
 * Handle a received ping by updating the counter
 */
int 
ping_receive_callback (void *arg) {
  pinData *pins = (pinData *) arg;
  std::cout << "Received a ping!" << std::endl;
  pins->curPing = std::time (nullptr);
  return EXIT_SUCCESS;
}

int main (int argc, char **argv)
{
  // Spawn an event loop
  GMainLoop *loop = g_main_loop_new(0, 0);  
  
  pinData *pins = (pinData *) malloc (sizeof (*pins));

  // Setup initial reset timer values
  pins->prevPing = std::time (nullptr);
  pins->curPing = std::time (&pins->prevPing);

  // Setup pins
  pins->ping_output = libsoc_gpio_request (PING_OUTPUT, LS_GPIO_SHARED);
  pins->ping_input = libsoc_gpio_request (PING_INPUT, LS_GPIO_SHARED);
  pins->reset_output = libsoc_gpio_request (RESET_OUTPUT, LS_GPIO_SHARED);


  // Make sure pins are actually requested
  if (pins->ping_output == NULL || pins->ping_input == NULL || 
      pins->reset_output == NULL)
  {
    goto fail;
  }

  // Set the pin directions
  libsoc_gpio_set_direction (pins->ping_output, OUTPUT);
  libsoc_gpio_set_direction (pins->ping_input, INPUT); 
  libsoc_gpio_set_direction (pins->reset_output, OUTPUT);

  // Check that directions were indeed set
  if (libsoc_gpio_get_direction (pins->ping_output) != OUTPUT) {
    printf ("Failed to set ping_output direction to OUTPUT\n");
    goto fail;
  }

  if (libsoc_gpio_get_direction(pins->ping_input) != INPUT)
  {
    printf ("Failed to set ping_input direction to INPUT\n");
    goto fail;
  }

  if (libsoc_gpio_get_direction(pins->reset_output) != OUTPUT)
  {
    printf ("Failed to set reset_output direction to OUTPUT\n");
    goto fail;
  }

  // Set the edge on the ping input pin to "falling" to detect interrupts
  libsoc_gpio_set_edge (pins->ping_input, FALLING);

  // Setup the ping callback to handle interrupts
  libsoc_gpio_callback_interrupt (pins->ping_input, 
                                  &ping_receive_callback,
                                  (void *) pins);
  
  // Add a timeout to the main loop to send pings at regular intervals
  g_timeout_add_seconds (PING_INTERVAL, sendPing, (gpointer) pins);

  // Run the main loop
  g_main_loop_run(loop); 

  fail: 
  
  // Clear the gpio memory
  if (pins->ping_output) {
    libsoc_gpio_free (pins->ping_output);
  }

  if (pins->ping_input) {
    libsoc_gpio_free (pins->ping_input);
  } 

  if (pins->reset_output) {
    libsoc_gpio_free (pins->reset_output);
  } 
}
