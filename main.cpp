#define DEBUG_USB_HOST
#define MAX_DUTY 0.3

#include "mbed.h"
#include <BTD.h>
#include <SPP.h>
#include <cstdint>

#include "motor_driver.h"
#include "translator.h"

USB usb(D11, D12, D13, A3, A2); // mosi, miso, sclk, ssel, intr
BTD device(&usb);
SPP bluetooth_serial(&device, "multi-board-mini");

float x,y;

// Motor pins.
PinName pwm_pin = PA_8, forward_pin = PB_0, backward_pin = PB_1;

// Motor2 pins.
PinName pwm_pin_2 = PA_9, forward_pin_2 = PB_6, backward_pin_2 = PB_7;

// LED
PwmOut led(PA_6);

AnalogIn psd(PA_5);

int main() {

  int received = bluetooth_serial.read();

  int dist;
  // Start server.
  if (usb.Init() == -1) {
    printf("Failed to start server.\r\n");
    while (true)
      ;
  }

  printf("Multi-Board-Console-Server started.\r\n");

  // Translator
  MultiBoardConsoleServer::Translator translator;

  // Motor driver.
  MotorDriver driver(forward_pin, backward_pin, pwm_pin);
  MotorDriver driver2(forward_pin_2, backward_pin_2, pwm_pin_2);

  int counter = 0;

  // Main process.
  while (1) {
    // printf("%d\n\r",dist);

    // Execute task once.
    usb.Task();

    if(!bluetooth_serial.connected){
      driver.drive(0);
      driver2.drive(0);
      continue;
    }

    counter++; //なくてもいいかも？
    if (counter % 100 == 0){
      dist = (1/psd.read()*7) - 6;
      // counter = (counter + 1 ) & 0xFF;
      // bluetooth_serial.write(0);
      // bluetooth_serial.write(counter);
      // bluetooth_serial.write(counter ^ 0);
      bluetooth_serial.write(2);
      bluetooth_serial.write(dist);
      bluetooth_serial.write(2 ^ dist); 

      counter = 0;   
    }

    delay(1);    
    
    // Process by the client request.
    while (bluetooth_serial.available() > 0) {
      int data = bluetooth_serial.read();

      // printf("received: %d\r\n", data);

      // Pass received data to the translator.
      translator.append(data);

      // Get data unit from translator if exists.
      while (translator.has_next()) {
        MultiBoardConsoleServer::DataUnit unit = translator.next();

        // Switch processes by the channel.
        switch (unit.channel) {
        case 1:
          led.write((float)unit.value);
          break;
        case 4:
          x = ((float)(2 * unit.value - 256) / 255.f);
          break;
        case 5:
          y = ((float)(2 * unit.value - 256) / 255.f);
          break;
        }
        // printf("%d: %d\r\n", unit.channel, unit.value);
        driver.drive(MAX_DUTY*(x+y));
        driver2.drive(MAX_DUTY*(y-x));
      }
    }
  }
}