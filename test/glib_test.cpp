#include <glib.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/wait.h>
#include <ctime>
#include <chrono>
#include "libsoc_gpio.h"
#include "libsoc_debug.h"

#define PING_OUTPUT 38
#define PING_INPUT 219 
#define RESET_OUTPUT 63 
#define PING_INTERVAL 1
#define RESET_INTERVAL 5
#define BOOT_INTERVAL 10
#define DEBUG_MESSAGES 1

using namespace std::chrono;

// Compilation instruction: 
//g++ -std=gnu++0x glib_test.cpp -o glibtest -I/usr/local/include -I/usr/include/glib-2.0 -I/usr/lib/aarch64-linux-gnu/glib-2.0/include -L/usr/local/lib -lglib-2.0 -lsoc

typedef struct {
  gpio *ping_output;
  gpio *ping_input;
  gpio *reset_output;
  milliseconds prevPing;
  milliseconds curPing;
  guint resetTimerId;
  guint bootTimerId;
} pinData;

/* Make some forward declarations so that we can access functions within one another */
static gboolean bootFinished (gpointer args);
static gboolean sendPing (gpointer args);
static gboolean checkReset (gpointer args);
static gboolean sendReset (gpointer args);
int receivePing (void *arg);

/**
 * After awaiting a boot for a specified amount of time, remove the timer source
 * and add a repeating timer for reset checks. 
 */
static gboolean
bootFinished (gpointer args) 
{
  if (DEBUG_MESSAGES == 1) {
    std::cout << "Finished awaiting boot. Removing boot timer." << std::endl;
  }

  // Cast args to the correct pointer type
  pinData *pins;
  pins = (pinData *) args;

  // Remove the bootTimer source
  g_source_remove (pins->bootTimerId);
  
  // Create a new resetTimer source
  pins->resetTimerId = g_timeout_add_seconds (RESET_INTERVAL,
                                              checkReset,
                                              (gpointer) pins);
  
  // Remove the bootTimerId
  pins->bootTimerId = 0; // TODO: might want to change these to pointers
  return TRUE;                             
}

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

/**
 * Reset timer callback. If no ping has been received, sends a reset signal
 * and creates a boot timer source, otherwise updates prev ping to current ping
 */
static gboolean
checkReset (gpointer args)
{
  // Cast args to the correct pointer type
  pinData *pins;
  pins = (pinData *) args;

  // Check if a ping has been received by comparing ping times
  if (pins->prevPing == pins->curPing) {
    if (DEBUG_MESSAGES == 1) {
      std::cout << "No ping receipt detected. Resetting..." << std::endl;
    }

    // No ping has been received. Send a reset signal
    sendReset (args);
    
    // Reset signal was sent. Remove the resetTimerId and reset timer source
    g_source_remove (pins->resetTimerId);
    pins->resetTimerId = 0; // TODO: might want to make this a pointer and set to NULL

    // Create a new boot timer and store its id
    pins->bootTimerId = g_timeout_add_seconds (BOOT_INTERVAL,
                                               bootFinished,
                                               (gpointer) pins);                                                
  } else {
    // A ping has been received. Update the prev ping to the current ping time
    // TODO: could this asynchronicity be messed up?
    pins->prevPing = pins->curPing;
    if (DEBUG_MESSAGES == 1) {
      std::cout << "Receipt detected. No reset necessary" << std::endl;
    }
  }

  return TRUE;
}

/**
 * Send a momentary reset signal
 */
static gboolean
sendReset (gpointer args)
{
  // Cast args to the correct pointer type
  pinData *pins;
  pins = (pinData *) args;

  // Send a reset signal
  libsoc_gpio_set_level (pins->reset_output, HIGH);
  // Unset the reset signal
  libsoc_gpio_set_level (pins->reset_output, LOW);

  return TRUE;
}

/**
 * Handle a received ping by updating the counter
 */
int 
receivePing (void *arg) {
  pinData *pins = (pinData *) arg; 
  milliseconds now = duration_cast< milliseconds >(
    system_clock::now().time_since_epoch()
  );
  if (DEBUG_MESSAGES == 1) {
    std::cout << "Received a ping!" << std::endl;
    long long nowMs = now.count();
    long long curMs = pins->curPing.count();
    std::cout << "Time difference: " << (nowMs - curMs) << std::endl;
  }
  pins->curPing = now;
  return EXIT_SUCCESS;
}

int main (int argc, char **argv)
{
  // Spawn an event loop
  GMainLoop *loop = g_main_loop_new(0, 0);  
  
  pinData *pins = (pinData *) malloc (sizeof (*pins));

  // Setup initial reset timer values
  // pins->prevPing = std::time (nullptr);
  // pins->curPing = std::time (&pins->prevPing);

  pins->prevPing = duration_cast< milliseconds >(
    system_clock::now().time_since_epoch()
  );
  pins->curPing = pins->prevPing;

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
                                  &receivePing,
                                  (void *) pins);
  
  // Add a timeout to the main loop to send pings at regular intervals
  // g_timeout_add_seconds (PING_INTERVAL, sendPing, (gpointer) pins);
  // libsoc_gpio_set_level (pins->ping_output, LOW);
  // Add a timeout to the main loop to check if a reset is necessary at 
  // regular intervals 
  pins->resetTimerId = g_timeout_add_seconds (RESET_INTERVAL, 
                                              checkReset,
                                              (gpointer) pins);

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
