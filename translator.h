#ifndef TRANSLATOR_H_
#define TRANSLATOR_H_

#ifndef MB_CONSOLE_BUFSIZE
#define MB_CONSOLE_BUFSIZE 10
#endif

namespace MultiBoardConsoleServer {

struct DataUnit {
  DataUnit(int channel = 0, int value = 0) {
    this->channel = channel;
    this->value = value;
  }

  int channel;
  int value;
};

class Translator {
  int _input_buffer[3];
  int _n_input_buffer_data;

  DataUnit _output_buffer[MB_CONSOLE_BUFSIZE];
  int _n_output_buffer_data;

public:
  Translator() {
    // Initialize input buffer.
    _input_buffer[0] = _input_buffer[1] = _input_buffer[2] = 0;
    _n_input_buffer_data = 0;

    // Initialize output buffer.
    _n_output_buffer_data = 0;
  }

  void append(int data) {
    // Append data to the tail of the input buffer.
    _input_buffer[2] = data;
    _n_input_buffer_data++;

    if (_n_input_buffer_data == 3) {
      // Unpack buffer.
      const int channel = _input_buffer[0];
      const int value = _input_buffer[1];
      const int terminator = _input_buffer[2];

      if ((channel ^ value) == terminator) {
        // Append data unit to the tail of the output buffer.
        if (_n_output_buffer_data < MB_CONSOLE_BUFSIZE) {
          _output_buffer[_n_output_buffer_data] = DataUnit(channel, value);
          _n_output_buffer_data++;
        }

        // Reset input buffer.
        _input_buffer[1] = 0;
        _input_buffer[2] = 0;
        _n_input_buffer_data = 0;
      } else {
        // Pop the head of the input buffer.
        _n_input_buffer_data--;
      }
    }

    // Forward the input buffer.
    _input_buffer[0] = _input_buffer[1];
    _input_buffer[1] = _input_buffer[2];
  }

  DataUnit next() {
    // Pop the head of the input buffer.
    DataUnit output = _output_buffer[0];
    _n_output_buffer_data--;

    // Staff the buffer.
    for (int i = 0; i < MB_CONSOLE_BUFSIZE - 1; i++) {
      _output_buffer[i] = _output_buffer[i + 1];
    }

    return output;
  }

  bool has_next() { return _n_output_buffer_data >= 1; }
};

} // namespace MultiBoardConsoleServer

#endif