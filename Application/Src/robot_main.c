#include "os.h"
#include "led.h"
#include "dbus.h"

led_id_t id = LED_1;
dbus_t remote;

void robot_main(void *args)
{
    (void)args;

    while (1) {
        dbus_get(&remote);

        led_set(LED_1, LED_1 == id ? true : false);
        led_set(LED_2, LED_2 == id ? true : false);
        led_set(LED_3, LED_3 == id ? true : false);
        id = (id + 1) % LAST_LED;
        os_delay(100);
    }
}
