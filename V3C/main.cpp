
/*
 *   Copyright (c) 2022 Martijn van Welie
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 *   https://github.com/weliem/bluez_inc
 */

#include <glib.h>
#include <stdio.h>
#include <signal.h>
#include <pthread.h>
#include "bt/adapter.h"
#include "bt/device.h"
#include "bt/logger.h"
#include "bt/agent.h"
#include "bt/application.h"
#include "bt/advertisement.h"
#include "bt/utility.h"
#include "bt/parser.h"

#include "drone.h"
#include "flightcontroller.h"
#include "testrunner.h"

#define TAG "Main"
#define HTS_SERVICE_UUID "00000001-710e-4a5b-8d75-3e5b444bc3cf"
#define DRONE_CHAR_UUID  "00000003-710e-4a5b-8d75-3e5b444bc3cf"
#define CUD_CHAR         "00002901-0000-1000-8000-00805f9b34fb"

GMainLoop *loop = NULL;
Adapter *default_adapter = NULL;
Advertisement *advertisement = NULL;
Application *app = NULL;

FlightController *pFlightController = NULL;
TestRunner *pTestRunner = NULL;
pthread_t theThread;

//Can't lock /var/run/pigpio.pid
//sudo killall pigpiod
static void process_drone_cmd(char *str) {
    char *sep = strstr(str, "=");
    if (sep == NULL)
        return;
    char *name = str;
    char *value= sep + 1;
    *sep = '\0';
    if (strcmp(name, "cmd") == 0) {
        pFlightController->onCommand(value);
        if (strcmp(value, "shutdown") == 0) {
            system("sudo shutdown now");
        }
    }
    else if (strcmp(name, "config") == 0) {
        pFlightController->onConfig(value);
    }
}

//pthread_create(&theThread, NULL, notify_drone_status, NULL);
static void * notify_drone_status(void *arg) {
    Application *theApp = (Application*)arg;
    while (binc_application_char_is_notifying(theApp, HTS_SERVICE_UUID, DRONE_CHAR_UUID)) {
        char *status = pFlightController->getStatus();
        GByteArray *byteArray = g_byte_array_sized_new(strlen(status));
        g_byte_array_append(byteArray, (const guint8 *)status, strlen(status));
        //??binc_application_set_char_value(theApp, HTS_SERVICE_UUID, DRONE_CHAR_UUID, byteArray);
        binc_application_notify(theApp, HTS_SERVICE_UUID, DRONE_CHAR_UUID, byteArray);
        g_byte_array_free(byteArray, TRUE);

        Utils::delayMs(1000);
    }
    return NULL;
}

void on_powered_state_changed(Adapter *adapter, gboolean state) {
    log_debug(TAG, "powered '%s' (%s)", state ? "on" : "off", binc_adapter_get_path(adapter));
}

void on_central_state_changed(Adapter *adapter, Device *device) {
    char *deviceToString = binc_device_to_string(device);
    log_debug(TAG, deviceToString);
    g_free(deviceToString);

    log_debug(TAG, "remote central %s is %s", binc_device_get_address(device), binc_device_get_connection_state_name(device));
    ConnectionState state = binc_device_get_connection_state(device);
    if (state == BINC_CONNECTED) {
        binc_adapter_stop_advertising(adapter, advertisement);
    } else if (state == BINC_DISCONNECTED){
        binc_adapter_start_advertising(adapter, advertisement);
    }
}

// This function is called when a read is done
// Use this to set the characteristic value if it is not set or to reject the read request
const char *on_local_char_read(const Application *application, const char *address, const char *service_uuid,
                        const char *char_uuid) {
    if (g_str_equal(service_uuid, HTS_SERVICE_UUID) && g_str_equal(char_uuid, DRONE_CHAR_UUID)) {
        /*const guint8 bytes[] = {0x06, 0x6f, 0x01, 0x00, 0xff, 0xe6, 0x07, 0x03, 0x03, 0x10, 0x04, 0x00, 0x01};
        GByteArray *byteArray = g_byte_array_sized_new(sizeof(bytes));
        g_byte_array_append(byteArray, bytes, sizeof(bytes));
        binc_application_set_char_value(application, service_uuid, char_uuid, byteArray);
        g_byte_array_free(byteArray, TRUE);*/
        return NULL;
    }
    return BLUEZ_ERROR_REJECTED;
}

// This function should be used to validate or reject a write request
const char *on_local_char_write(const Application *application, const char *address, const char *service_uuid,
                          const char *char_uuid, GByteArray *byteArray) {
    GString *result = g_byte_array_as_hex(byteArray);
    log_debug(TAG, "write request characteristic <%s> with value <******%s******>", char_uuid, byteArray->data);
    
    process_drone_cmd((char*)byteArray->data);
    
    g_string_free(result, TRUE);

    return NULL;
}

// This function is called after a write request was validates and the characteristic value was set
void on_local_char_updated(const Application *application, const char *service_uuid,
                           const char *char_uuid, GByteArray *byteArray) {
    GString *result = g_byte_array_as_hex(byteArray);
    log_debug(TAG, "characteristic <%s> updated to <%s>", char_uuid, result->str);
    g_string_free(result, TRUE);
}

void on_local_char_start_notify(const Application *application, const char *service_uuid, const char *char_uuid) {
    log_debug(TAG, "on start notify");
    if (g_str_equal(service_uuid, HTS_SERVICE_UUID) && g_str_equal(char_uuid, DRONE_CHAR_UUID)) {
        /*
        const guint8 bytes[] = {0x06, 0x6A, 0x01, 0x00, 0xff, 0xe6, 0x07, 0x03, 0x03, 0x10, 0x04, 0x00, 0x01};
        GByteArray *byteArray = g_byte_array_sized_new(sizeof(bytes));
        g_byte_array_append(byteArray, bytes, sizeof(bytes));
        binc_application_notify(application, service_uuid, char_uuid, byteArray);
        g_byte_array_free(byteArray, TRUE);
        */
        pthread_create(&theThread, NULL, notify_drone_status, (void*)application);
    }
}

void on_local_char_stop_notify(const Application *application, const char *service_uuid, const char *char_uuid) {
    log_debug(TAG, "on stop notify");
}

gboolean callback(gpointer data) {
    if (app != NULL) {
        binc_adapter_unregister_application(default_adapter, app);
        binc_application_free(app);
        app = NULL;
    }

    if (advertisement != NULL) {
        binc_adapter_stop_advertising(default_adapter, advertisement);
        binc_advertisement_free(advertisement);
    }

    if (default_adapter != NULL) {
        binc_adapter_free(default_adapter);
        default_adapter = NULL;
    }

    g_main_loop_quit((GMainLoop *) data);
    return FALSE;
}

static void cleanup_handler(int signo) {
    if (signo == SIGINT) {
        if (pFlightController)
            callback(loop);
        if (pTestRunner)
            pTestRunner->stop();
    }
}

int main(int argc, char **argv) {
    gpioInitialise();
    
    log_set_filename("drone_log", 0, 0);
    
    int mode = MODE_INDOOR;
    string component = "gps";
    if ((argc >= 3) && (strcmp(argv[1], "-m") == 0)) {
        if (strcmp(argv[2], "indoor") == 0) {
            mode = MODE_INDOOR;
        }
        else if (strcmp(argv[2], "outdoor") == 0) {
            mode = MODE_OUTDOOR;
        }
        else if (strcmp(argv[2], "training") == 0) {
            mode = MODE_TRAINING;
        }
        else if (strcmp(argv[2], "test") == 0) {
            mode = MODE_TEST;
            if (argc >= 4)
                component = argv[3];
        }
        else {
            cout << "Unknown mode " << argv[1] << endl;
        }
    }
    if (mode == MODE_TEST) {
        signal(SIGINT, cleanup_handler);
        
        pTestRunner = new TestRunner(component);
        pTestRunner->run();

        delete pTestRunner;
    }
    else {
        pFlightController = new FlightController(mode);
        
        // Get a DBus connection
        GDBusConnection *dbusConnection = g_bus_get_sync(G_BUS_TYPE_SYSTEM, NULL, NULL);

        // Setup handler for CTRL+C
        if (signal(SIGINT, cleanup_handler) == SIG_ERR)
            log_error(TAG, "can't catch SIGINT");

        // Setup mainloop
        loop = g_main_loop_new(NULL, FALSE);

        // Get the default default_adapter
        default_adapter = binc_adapter_get_default(dbusConnection);

        if (default_adapter != NULL) {
            log_debug(TAG, "using default_adapter '%s'", binc_adapter_get_path(default_adapter));

            // Make sure the adapter is on
            binc_adapter_set_powered_state_cb(default_adapter, &on_powered_state_changed);
            if (!binc_adapter_get_powered_state(default_adapter)) {
                binc_adapter_power_on(default_adapter);
            }

            // Setup remote central connection state callback
            binc_adapter_set_remote_central_cb(default_adapter, &on_central_state_changed);

            // Setup advertisement
            GPtrArray *adv_service_uuids = g_ptr_array_new();
            g_ptr_array_add(adv_service_uuids, (gpointer)HTS_SERVICE_UUID);

            advertisement = binc_advertisement_create();
            binc_advertisement_set_local_name(advertisement, "Drone");
            binc_advertisement_set_interval(advertisement, 500, 500);
            binc_advertisement_set_tx_power(advertisement, 5);
            binc_advertisement_set_services(advertisement, adv_service_uuids);
            g_ptr_array_free(adv_service_uuids, TRUE);
            binc_adapter_start_advertising(default_adapter, advertisement);

            // Start application
            app = binc_create_application(default_adapter);
            binc_application_add_service(app, HTS_SERVICE_UUID);
            binc_application_add_characteristic(
                    app,
                    HTS_SERVICE_UUID,
                    DRONE_CHAR_UUID,
                    GATT_CHR_PROP_INDICATE | GATT_CHR_PROP_WRITE);
            binc_application_add_descriptor(
                    app,
                    HTS_SERVICE_UUID,
                    DRONE_CHAR_UUID,
                    CUD_CHAR,
                    GATT_CHR_PROP_READ | GATT_CHR_PROP_WRITE);

            const guint8 cud[] = "Flying Hawk Drone Flight Controller";
            GByteArray *cudArray = g_byte_array_sized_new(sizeof(cud));
            g_byte_array_append(cudArray, cud, sizeof(cud));
            binc_application_set_desc_value(app, HTS_SERVICE_UUID, DRONE_CHAR_UUID, CUD_CHAR, cudArray);

            binc_application_set_char_read_cb(app, &on_local_char_read);
            binc_application_set_char_write_cb(app, &on_local_char_write);
            binc_application_set_char_start_notify_cb(app, &on_local_char_start_notify);
            binc_application_set_char_stop_notify_cb(app, &on_local_char_stop_notify);
            binc_application_set_char_updated_cb(app, &on_local_char_updated);
            binc_adapter_register_application(default_adapter, app);
        } else {
            log_debug("MAIN", "No default_adapter found");
        }

        // Bail out after some time
        //g_timeout_add_seconds(600, callback, loop);

        // Start the mainloop
        g_main_loop_run(loop);

        // Clean up mainloop
        g_main_loop_unref(loop);

        // Disconnect from DBus
        g_dbus_connection_close_sync(dbusConnection, NULL, NULL);
        //?????g_object_unref(dbusConnection);
        app = NULL;

        pFlightController->cmdStop();
        delete pFlightController;
    }//drone mode
    gpioTerminate();
    
    //close the log file
    log_enabled(false);
    

    return 0;
}
