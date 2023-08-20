# Accelerometer Client

The Accelerometer client provides an interface to the sensor board, using serial
communication and implementing the commands to configure the board, check status
and return samples from the ADC. 

The client consists of a `Controller`, which provides access to the board. The
`Controller` can be instanciated and called directly for one-shot sampling,
board configuration, and status reporting. It also provides a method for 
free-running sample collection, which will write data back to a caller-provided
`SampleSink`. This is designed for execution in a separate thread and supports
user or caller termination. 

## Examples

Useage examples can be found in the `./examples` path. These include a basic
one-shot read example and a free-running sample collection that writes data to
a CSV file. 

The following shows a simple example of reading the board ID. In this example, 
the board is connected on the serial port `/dev/ttyACM0`.

```python
import board
mon = board.Controller('/dev/ttyACM0')
b_id, accel_type = mon.board_id()
print(f"Board ID: {b_id}, Accelerometer: {accel_type}")
```
