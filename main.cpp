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
SPP bluetooth_serial(&device, "Multi-Board-Mini");

float x,y;

// Motor pins.
//PinName pwm_pin = PA_8, forward_pin = PB_0, backward_pin = PB_1;

// Motor2 pins.
//PinName pwm_pin_2 = PA_9, forward_pin_2 = PB_6, backward_pin_2 = PB_7;

// LED
//DigitalOut LED_1(PA_6);

//IO
AnalogIn psd(PA_6);

int main() {

  int received = bluetooth_serial.read();

  int dist;
  // Start server.
  if (usb.Init() == -1) {
    printf("Failed to start server.\r\n");
    while (true);
  }
  printf("Multi-Board-Console-Server started.\r\n");

  // Translator
  MultiBoardConsoleServer::Translator translator;

  // Motor driver.
  //MotorDriver driver(forward_pin, backward_pin, pwm_pin);
  //MotorDriver driver2(forward_pin_2, backward_pin_2, pwm_pin_2);


  // Main process.
  while (1) {
    // Execute task once.
    //printf("%d\n\r",dist);
    usb.Task();
    //delay(10);
    dist = (1/psd.read()*7);

    // Process by the client requist.
    if (bluetooth_serial.available()) {
    //while (received != -1) {  
      // Pass received data to the translator.
      translator.append(bluetooth_serial.read());
      
      //translator.append(received);
      //received = bluetooth_serial.read();
    
      // Get data unit from translator if exists.
      while (translator.has_next()) {
        MultiBoardConsoleServer::DataUnit unit = translator.next();
        // Switch processes by the channel.
        switch (unit.channel) {
        case 1:
          LED_1.write(unit.value);
          break;
        case 4:
          x = ((float)(2 * unit.value - 256) / 255.f);
          break;
        case 5:
          y = ((float)(2 * unit.value - 256) / 255.f);
          break;
        }

        //driver.drive(MAX_DUTY*(x+y));
        //driver2.drive(MAX_DUTY*(y-x));
      }
    }
    bluetooth_serial.write(2);  
    bluetooth_serial.write(dist);
    bluetooth_serial.write(2 ^ dist);
      
  }
}
