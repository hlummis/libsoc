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
 *  - Log all suspicious events (i.e. resets) in a log file where the records
 *    can be accessed later. 
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
 * 
 * TODO: add check after reset to see if reset was successful
 */
#define BOOST_LOG_DYN_LINK 1

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
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

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
#define LOG_FNAME_PREFIX "leroy"

// Define namespaces
namespace logging = boost::log;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
namespace keywords = boost::log::keywords;

using namespace std::chrono;
using namespace logging::trivial;

// Compilation instruction: 
// g++ -std=c++11 glib_test.cpp -o glibtest -lboost_log `pkg-config --cflags --libs libsoc glib-2.0` -lboost_thread -lpthread -lboost_system -lboost_log_setup

typedef struct {
  gpio *ping_output;     // Stores the address of the ping output pin object
  gpio *ping_input;      // Stores the address of the ping input pin object
  gpio *reset_output;    // Stores the address of the reset pin object
  milliseconds prevPing; // Stores the time of the ping last time reset checked
  milliseconds curPing;  // Stores the time of the most recent ping
  guint resetTimerId;    // Stores the id of the reset timer source
  guint bootTimerId;     // Stores the id of the boot timer source
  src::severity_logger<severity_level> *lg; // Stores a pointer to the logger
  long long lastResetTime; // Store the time of the last reset
} pinData;

/* Make some forward declarations so that we can access functions within one another */
static gboolean bootFinished (gpointer args);
static gboolean sendPing (gpointer args);
static gboolean checkReset (gpointer args);
static gboolean sendReset (gpointer args);
int receivePing (void *arg);
void init_logs ();

/**
 * Initialize loggers
 */
void
init_logs () 
{
  // Create the file log
  logging::add_file_log(
    keywords::file_name = LOG_FNAME_PREFIX "_%N.log",
    keywords::rotation_size = 10 * 1024 * 1024,
    keywords::time_based_rotation = sinks::file::rotation_at_time_point(0, 0, 0),
    keywords::format = "[%TimeStamp%]: %Message%",
    keywords::auto_flush = true
  );
  
  // Set the filter
  logging::core::get()->set_filter(
    logging::trivial::severity >= logging::trivial::info
  );
}

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
  using namespace logging::trivial;

  // Cast args to the correct pointer type
  pinData *pins;
  pins = (pinData *) args;

  // Check if a ping has been received by comparing ping times
  if (pins->prevPing == pins->curPing) {
    if (DEBUG_MESSAGES == 1) {
      std::cout << "No ping receipt detected. Resetting..." << std::endl;
    }
    
    // Determine epoch time of last ping (in seconds)
    long long curMs = pins->curPing.count() / 1000;
    
    // If no ping has been detected since the last reset, we have a major
    // problem
    if (pins->lastResetTime && curMs < pins->lastResetTime) {
      BOOST_LOG_SEV (*pins->lg, error) << "No ping detected since last " <<
        "reset. Last ping detected at " << curMs << ". Last reset detected " <<
        "at " << pins->lastResetTime << ". Sending reset signal."; 
    } else {
      // Log a record of the last ping at reset time
      BOOST_LOG_SEV(*pins->lg, warning) << 
        "No ping detected within timeframe. Last ping detected at " << curMs << 
        " seconds since epoch. Sending reset signal.";
    }
    
    // Update the reset time
    pins->lastResetTime = duration_cast< seconds >(
      system_clock::now().time_since_epoch()
    ).count();
      
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
  if (diff < PING_BOUNCETIME) {
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
  // Setup logger
  init_logs ();
  logging::add_common_attributes ();
    
  src::severity_logger<severity_level> lg;

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

  // Add pointer to logger to pins
  pins->lg = &lg;

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
    std::string logMessage = "Failed to set ping_output direction to OUTPUT\n";
    if (DEBUG_MESSAGES == 1) {
      std::cout << logMessage << std::endl;
    }
    BOOST_LOG_SEV(lg, fatal) << logMessage;
    goto fail;
  }

  if (libsoc_gpio_get_direction(pins->ping_input) != INPUT)
  {
    std::string logMessage = "Failed to set ping_input direction to INPUT\n";
    if (DEBUG_MESSAGES == 1) {
      std::cout << logMessage << std::endl;
    }
    BOOST_LOG_SEV(lg, fatal) << logMessage;
    goto fail;
  }

  if (libsoc_gpio_get_direction(pins->reset_output) != OUTPUT)
  {
    std::string logMessage = "Failed to set reset_output direction to OUTPUT\n";
    if (DEBUG_MESSAGES == 1) {
      std::cout << logMessage << std::endl;
    }
    BOOST_LOG_SEV(lg, fatal) << logMessage;
    goto fail;
  }

  // Set the edge on the ping input pin to "falling" to detect interrupts
  libsoc_gpio_set_edge (pins->ping_input, FALLING);

  // Setup the ping callback to handle interrupts
  libsoc_gpio_callback_interrupt (pins->ping_input, 
                                  &receivePing,
                                  (void *) pins);
  
  // Add a timeout to the main loop to send pings at regular intervals
  g_timeout_add_seconds (PING_INTERVAL, sendPing, (gpointer) pins);
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
