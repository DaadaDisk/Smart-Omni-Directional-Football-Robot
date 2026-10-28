import logging
import time
import threading
from typing import Optional, Union, Callable
import serial
import serial.tools.list_ports


class ArduinoSerial:
    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = 115200,
        timeout: float = 1.0,
        write_timeout: float = 1.0,
        auto_reconnect: bool = True,
        max_reconnect_attempts: int = 5,
        reconnect_delay: float = 2.0,
        use_background_writer: bool = False,
        log_level: int = logging.INFO,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.auto_reconnect = auto_reconnect
        self.max_reconnect_attempts = max_reconnect_attempts
        self.reconnect_delay = reconnect_delay
        self.use_background_writer = use_background_writer

        self._serial: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._connected = False
        self._reconnect_count = 0

        # Background writer thread
        self._writer_thread: Optional[threading.Thread] = None
        self._write_queue: list = []
        self._stop_writer = False
        self._writer_condition = threading.Condition()

        # Setup logging
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.setLevel(log_level)
        if not self.logger.handlers:
            ch = logging.StreamHandler()
            ch.setLevel(log_level)
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            ch.setFormatter(formatter)
            self.logger.addHandler(ch)

        # Autodetect port if not provided
        if self.port is None:
            self.port = self._auto_detect_port()
            if self.port:
                self.logger.info(f"Autodetected port: {self.port}")
            else:
                self.logger.warning("No port specified and autodetection failed.")

        # self.open() #we open on demand

    def _auto_detect_port(self) -> Optional[str]:
        """Try to find a serial port that looks like an Arduino."""
        ports = serial.tools.list_ports.comports()
        for p in ports:
            # Common Arduino VID/PID or description
            if 'Arduino' in p.description or 'USB' in p.description:
                return p.device
        # Fallback: return first available port (if any)
        if ports:
            return ports[0].device
        return None

    def open(self) -> bool:
        if self._connected and self._serial and self._serial.is_open:
            return True

        with self._lock:
            if self.port is None:
                self.logger.error("No port specified.")
                return False

            try:
                self._serial = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=self.timeout,
                    write_timeout=self.write_timeout,
                )
                self._connected = True
                self._reconnect_count = 0
                self.logger.info(f"Connected to {self.port} at {self.baudrate} baud.")

                # Start background writer if enabled
                if self.use_background_writer and self._writer_thread is None:
                    self._start_writer_thread()

                return True
            except serial.SerialException as e:
                self.logger.error(f"Failed to open port {self.port}: {e}")
                self._connected = False
                return False

    def close(self) -> None:
        """Close the serial connection and stop background threads."""
        with self._lock:
            if self._writer_thread and self._writer_thread.is_alive():
                self._stop_writer = True
                with self._writer_condition:
                    self._writer_condition.notify_all()
                self._writer_thread.join(timeout=2.0)
                self._writer_thread = None

            if self._serial and self._serial.is_open:
                self._serial.close()
                self._connected = False
                self.logger.info("Serial port closed.")

    def reconnect(self) -> bool:
        """Attempt to reconnect after a failure."""
        self.close()
        time.sleep(self.reconnect_delay * (2 ** self._reconnect_count))
        self._reconnect_count += 1
        return self.open()

    def send(self, data: Union[str, bytes], append_newline: bool = True) -> bool:
        """
        Send data to the Arduino.

        :param data: String or bytes to send.
        :param append_newline: If True and data is string, append '\n' (default Arduino line ending).
        :return: True if successful.
        """
        if not self._connected and not self.open():
            self.logger.error("Cannot send: not connected.")
            return False

        # Convert to bytes
        if isinstance(data, str):
            if append_newline:
                data += '\n'
            data = data.encode('utf-8')
        elif isinstance(data, bytes):
            if append_newline:
                data += b'\n'
        else:
            raise TypeError("data must be str or bytes")

        if self.use_background_writer:
            # Queue for background sending
            with self._writer_condition:
                self._write_queue.append(data)
                self._writer_condition.notify()
            return True
        else:
            # Synchronous send
            return self._write_bytes(data)

    def _write_bytes(self, data: bytes) -> bool:
        """Low-level synchronous write with error handling and auto-reconnect."""
        try:
            with self._lock:
                if not self._serial or not self._serial.is_open:
                    if self.auto_reconnect and self._reconnect_count < self.max_reconnect_attempts:
                        self.logger.warning("Connection lost, attempting reconnect...")
                        if self.reconnect():
                            return self._write_bytes(data)  # recursive retry
                    return False
                self._serial.write(data)
                self._serial.flush()
                return True
        except (serial.SerialException, OSError) as e:
            self.logger.error(f"Write error: {e}")
            self._connected = False
            if self.auto_reconnect and self._reconnect_count < self.max_reconnect_attempts:
                if self.reconnect():
                    return self._write_bytes(data)
            return False

    def receive(self, size: int = 1) -> Optional[bytes]:

        if not self._connected and not self.open():
            self.logger.error("Cannot receive: not connected.")
            return None

        try:
            with self._lock:
                if not self._serial or not self._serial.is_open:
                    return None
                return self._serial.read(size)
        except serial.SerialException as e:
            self.logger.error(f"Read error: {e}")
            self._connected = False
            if self.auto_reconnect and self._reconnect_count < self.max_reconnect_attempts:
                self.reconnect()
            return None

    def receive_line(self) -> Optional[str]:
        if not self._connected and not self.open():
            self.logger.error("Cannot receive: not connected.")
            return None

        try:
            with self._lock:
                if not self._serial or not self._serial.is_open:
                    return None
                line = self._serial.readline()
                if line:
                    return line.decode('utf-8').rstrip('\r\n')
                return None
        except serial.SerialException as e:
            self.logger.error(f"Readline error: {e}")
            self._connected = False
            if self.auto_reconnect and self._reconnect_count < self.max_reconnect_attempts:
                self.reconnect()
            return None

    def flush_input(self) -> None:
        """Flush input buffer."""
        if self._serial and self._serial.is_open:
            self._serial.reset_input_buffer()

    def flush_output(self) -> None:
        """Flush output buffer."""
        if self._serial and self._serial.is_open:
            self._serial.reset_output_buffer()

    def _start_writer_thread(self) -> None:
        """Start the background writer thread."""
        self._stop_writer = False
        self._writer_thread = threading.Thread(target=self._writer_loop, daemon=True)
        self._writer_thread.start()
        self.logger.info("Background writer thread started.")

    def _writer_loop(self) -> None:
        """Background thread that processes the write queue."""
        while not self._stop_writer:
            data = None
            with self._writer_condition:
                if not self._write_queue:
                    self._writer_condition.wait(timeout=0.5)
                    continue
                data = self._write_queue.pop(0)

            if data is not None:
                # Attempt to send; if fails, requeue or drop?
                # For simplicity, we drop on failure after a single attempt
                # (could implement retry logic)
                if not self._write_bytes(data):
                    self.logger.warning("Background write failed, data dropped.")
                # else: successfully sent

        self.logger.info("Background writer stopped.")

    # Context manager methods
    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    @property
    def is_connected(self) -> bool:
        """Check if the connection is currently active."""
        return self._connected and self._serial and self._serial.is_open

    @property
    def port_name(self) -> Optional[str]:
        """Return the current port name."""
        return self.port


if __name__ == "__main__":

    arduino = ArduinoSerial()
    arduino.open()
    import time
    time.sleep(1)
    while True:
        x = input('>')
        if 'q' in x.lower():
            break
        w = [float(i) for i in x.split(' ')]
        line = f"{w[0]:.2f},{w[1]:.2f},{w[2]:.2f}\n"
        print(line)
        arduino.send(line)

    arduino.close()
