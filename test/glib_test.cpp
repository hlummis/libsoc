/**
 * The purpose of this file is to define a behavior for the Jetson wherein 
 * the Jetson will...
 *  - Send outbound pings on a GPIO pin at a regular interval of PING_INTERVAL
 *    seconds.
 *  - Receive inbound pings on an interrupt basis (i.e. through threaded
 *    polling). 
 *  - Send reset signals when appropriate. Appropriate here is defined to be
 *    when the Jetson has not received an inbound ping within the time 
 *    interval defined to be RESET_INTERVAL seconds. When no ping is detected
 *    within that timeframe, the Jetson will assume the chip that is expected
 *    to send such pings has been deactivated and will attempt to reset it in
 *    response. 
 *  - Await boots after resetting. The Jetson will not immediately expect to 
 *    begin receiving inbound pings again upon reset. Rather, the Jetson will
 *    continue simply sending pings for a number of seconds defined by 
 *    BOOT_INTERVAL. Upon the completion of this interval, the Jetson will 
 *    again expect to receive inbound pings.
 *  - Logs all suspicious events (i.e. required resets) in a separate file
 *    for debugging purposes
 * 
 * This program depends on the following libraries:
 * 
 ********************************* glib-2.0 ************************************
 *   GLib allows for simplified creation of an event loop so that asynchronous
 *   behavior may occur within a single thread context. The main use of this 
 *   event loop here is to coordinate timeout events. Interrupt detection occurs
 *   in a separate thread.
 ******************************************************************************* 
 ********************************** libsoc *************************************
 * libsoc provides a simple C interface for working with the Jetson's GPIO pins.
 * Lighter-weight libraries were tested, however this particular library provides
 * a threaded callback-based implementation of edge detection on the GPIO pins,
 * so we opted for it instead.
 ******************************************************************************* 
 *********************************** boost *************************************
 * The boost library provides a simple interface for handling logs. Without
 * boost, tasks such as creating new log files on a regular basis would be 
 * rather difficult.
 ******************************************************************************* 
 * All GPIO interfacing is accomplished through libsoc while any events of note 
 * are handled within the main event loop.
 * 
 * Approximately 4K Ohm resistance should be used between the Pi ping output and
 * the Jetson ping input to avoid noise-triggered interrupts.
 */
#include <glib.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/wait.h>
#include <chrono>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

#include "libsoc_gpio.h"
#include "libsoc_debug.h"

#define PING_OUTPUT 38
#define PING_INPUT 219 
#define RESET_OUTPUT 63 
#define PING_INTERVAL 1
#define RESET_INTERVAL 5
#define BOOT_INTERVAL 10
#define DEBUG_MESSAGES 1
#define PING_BOUNCETIME 300

// Define namespaces
namespace logging = boost::log;
using namespace std::chrono;

// Compilation instruction: 
//g++ -std=gnu++0x glib_test.cpp -o glibtest -I/usr/local/include -I/usr/include/glib-2.0 -I/usr/lib/aarch64-linux-gnu/glib-2.0/include -L/usr/local/lib -lglib-2.0 -lsoc

typedef struct {
  gpio *ping_output;     // Stores the address of the ping output pin object
  gpio *ping_input;      // Stores the address of the ping input pin object
  gpio *reset_output;    // Stores the address of the reset pin object
  milliseconds prevPing; // Stores the time of the ping last time reset checked
  milliseconds curPing;  // Stores the time of the most recent ping
  guint resetTimerId;    // Stores the id of the reset timer source
  guint bootTimerId;     // Stores the id of the boot timer source
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
  // Check to see if receipt occurred within bouncetime
  long long nowMs = now.count();
  long long curMs = pins->curPing.count();
  int diff = (int) (nowMs - curMs);

  // Don't update curPing if difference is less than bouncetime
  if (diff < BOUNCE_TIME) {
    return EXIT_SUCCESS;
  }

  pins->curPing = now;

  if (DEBUG_MESSAGES == 1) {
    std::cout << "Received a ping!" << std::endl;    
    std::cout << "Time difference: " << (nowMs - curMs) << std::endl;
  }  
  return EXIT_SUCCESS;
}

int main (int argc, char **argv)
{
  // Spawn an event loop
  GMainLoop *loop = g_main_loop_new(0, 0);  
  
  pinData *pins = (pinData *) malloc (sizeof (*pins));

  // Setup initial reset timer values
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
