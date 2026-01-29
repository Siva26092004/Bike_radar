import serial
import time
from pathlib import Path
import struct
import numpy as np
import logging
import threading
from PySide6.QtCore import QObject, Signal, Slot

CONFIG_BAUD = 115200
MAGIC_WORD = bytes([0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07])
HEADER_STRUCT = struct.Struct('<Q8I')  # little-endian, matches device header
HEADER_LEN = HEADER_STRUCT.size

log = logging.getLogger(__name__)


class GridBackend(QObject):
    grid_ready = Signal(object)
    radar_points_ready = Signal(object)  # Emits rescaled points [(x, y, target_id), ...]

    def __init__(self):
        super().__init__()
        self.reading_thread = None
        self.running = False
        
        # Grid dimensions (set when create_grid is called)
        self.x_min = 0
        self.x_max = 0
        self.y_min = 0
        self.y_max = 0
        self.nx = 0
        self.ny = 0

    @Slot(dict)
    def create_grid(self, cfg):
        try:
            self.x_min = cfg["x_min"]
            self.x_max = cfg["x_max"]
            self.y_min = cfg["y_min"]
            self.y_max = cfg["y_max"]
            dx = cfg["dx"]
            dy = cfg["dy"]

            # Validation
            if dx <= 0 or dy <= 0:
                print("Invalid cell size")
                return
            if self.x_max <= self.x_min or self.y_max <= self.y_min:
                print("Invalid range")
                return

            self.nx = int((self.x_max - self.x_min) / dx)
            self.ny = int((self.y_max - self.y_min) / dy)

            if self.nx <= 0 or self.ny <= 0:
                print("Invalid grid size")
                return

            # Y rows, X columns
            grid = np.zeros((self.ny, self.nx), dtype=np.float32)

            print(f"Grid dimensions stored: X[{self.x_min}, {self.x_max}], Y[{self.y_min}, {self.y_max}]")
            print(f"Grid cells: {self.nx} x {self.ny}")
            
            self.grid_ready.emit(grid)

        except Exception as e:
            print("Backend error:", e)

    def send_config(self, config_port: str, config_file: Path):
        """Send configuration commands to the radar via the config serial port."""
        print(f"Opening config port {config_port} at {CONFIG_BAUD} baud")
        try:
            with serial.Serial(config_port, CONFIG_BAUD, timeout=1) as ser:
                time.sleep(0.5)  # Let port stabilize
                print(f"Reading config from {config_file}")
                with config_file.open('r') as f:
                    for line in f:
                        line = line.strip()
                        if not line or line.startswith('%'):
                            continue
                        cmd = line + '\n'
                        print(f"SEND: {cmd.strip()}")
                        ser.write(cmd.encode('utf-8'))
                        time.sleep(0.1)  # Small delay between commands
                        # Read and echo all available response bytes for this command
                        time.sleep(0.05)
                        resp_chunks = []
                        while ser.in_waiting:
                            resp_chunks.append(ser.read(ser.in_waiting))
                            time.sleep(0.01)
                        if resp_chunks:
                            response = b''.join(resp_chunks).decode('utf-8', errors='ignore').strip()
                            if response:
                                print(f"RESP: {response}")

                print("Configuration sent; waiting briefly before reading data")
                time.sleep(0.5)
        except Exception as e:
            print(f"Error sending config: {e}")

    def start_reading(self, data_port: str, baud_rate: int = 921600):
        """Start continuous radar data reading in a separate thread."""
        if self.reading_thread and self.reading_thread.is_alive():
            print("Already reading from radar")
            return

        self.running = True
        self.reading_thread = threading.Thread(
            target=self._read_from_serial_port,
            args=(data_port, baud_rate),
            daemon=True
        )
        self.reading_thread.start()
        print(f"Started radar reading thread on {data_port}")

    def stop_reading(self):
        """Stop the radar reading thread."""
        self.running = False
        if self.reading_thread:
            self.reading_thread.join(timeout=2)
            print("Stopped radar reading thread")

    def _read_from_serial_port(self, port: str, baud_rate: int):
        """Continuous radar data reading (runs in thread)."""
        try:
            with serial.Serial(port, baud_rate, timeout=2) as ser:
                buffer = bytearray()
                while self.running:
                    # Read available data
                    if ser.in_waiting > 0:
                        buffer.extend(ser.read(ser.in_waiting))

                    # Look for magic word
                    magic_idx = buffer.find(MAGIC_WORD)
                    if magic_idx == -1:
                        # No magic word found, keep first 1KB and continue reading
                        if len(buffer) > 1024:
                            buffer = buffer[-1024:]
                        time.sleep(0.01)
                        continue
                    
                    # Remove data before magic word
                    buffer = buffer[magic_idx:]
                    
                    # Check if we have enough for header
                    if len(buffer) < HEADER_LEN:
                        time.sleep(0.01)
                        continue
                    
                    # Parse header to get frame length
                    try:
                        magic, version, totalPacketLen, *_ = HEADER_STRUCT.unpack(buffer[:HEADER_LEN])
                        if magic != 0x0708050603040102:  # Verify magic word
                            log.warning("Invalid magic word, skipping byte")
                            buffer = buffer[1:]
                            continue
                    except struct.error:
                        time.sleep(0.01)
                        continue
                    
                    # Check if full frame is available
                    if len(buffer) < totalPacketLen:
                        time.sleep(0.01)
                        continue
                    
                    # Extract frame
                    frame_bytes = bytes(buffer[:totalPacketLen])
                    buffer = buffer[totalPacketLen:]
                    
                    # Parse the frame
                    self.parse_standard_frame(frame_bytes)

        except Exception as e:
            print(f"Error reading from serial port: {e}")
        finally:
            self.running = False

    def parse_standard_frame(self, frameData):
        """Parse a complete radar frame."""
        headerStruct = 'Q8I'
        frameHeaderLen = struct.calcsize(headerStruct)
        tlvHeaderLength = 8
        
        try:
            # Read in frame Header
            magic, version, totalPacketLen, platform, frameNum, timeCPUCycles, numDetectedObj, numTLVs, subFrameNum = struct.unpack(headerStruct, frameData[:frameHeaderLen])
        except:
            log.error('Error: Could not read frame header')
            return
        
        offset = frameHeaderLen
        for i in range(numTLVs):
            try:
                tlvType, tlvLength = self.tlv_header_decode(frameData[offset:offset+tlvHeaderLength])
                tlvPayload = frameData[offset + tlvHeaderLength : offset+tlvHeaderLength+tlvLength]
                
                if tlvType == 1020:
                    # CompressedSphericalPointCloudTLV
                    pass
                elif tlvType == 1010:
                    # TrackTLV - this is what we want for XY coordinates
                    numDetectedTargets, targets = self.parse_track_tlv(tlvPayload, tlvLength)
                    if numDetectedTargets > 0:
                        # Rescale and emit points
                        self.rescale_and_emit_points(targets)
                elif tlvType == 1011:
                    # TargetIndexTLV
                    pass
                elif tlvType == 1012:
                    # TrackHeightTLV
                    pass
                elif tlvType == 1021:
                    # Presence Indication
                    pass

            except Exception as e:
                log.error(f'TLV parsing error for TLV {i+1}/{numTLVs}: {e}')
                break  # Stop processing this frame on error
            
            offset += tlvHeaderLength + tlvLength

    def parse_track_tlv(self, tlvData, tlvLength):
        """Parse Track TLV to extract target positions."""
        targetStruct = 'I27f'
        targetSize = struct.calcsize(targetStruct)
        numDetectedTargets = int(tlvLength/targetSize)
        targets = np.empty((numDetectedTargets, 16))
        
        for i in range(numDetectedTargets):
            try:
                targetData = struct.unpack(targetStruct, tlvData[:targetSize])
            except:
                print('ERROR: Target TLV parsing failed')
                return 0, targets

            targets[i, 0] = targetData[0]   # Target ID
            targets[i, 1] = targetData[1]   # X Position
            targets[i, 2] = targetData[2]   # Y Position
            targets[i, 3] = targetData[3]   # Z Position
            targets[i, 4] = targetData[4]   # X Velocity
            targets[i, 5] = targetData[5]   # Y Velocity
            targets[i, 6] = targetData[6]   # Z Velocity
            targets[i, 7] = targetData[7]   # X Acceleration
            targets[i, 8] = targetData[8]   # Y Acceleration
            targets[i, 9] = targetData[9]   # Z Acceleration
            targets[i, 10] = targetData[26] # G
            targets[i, 11] = targetData[27] # Confidence Level
            print(f" pos X {targets[i,1]} pos Y {targets[i,2]} pos Z {targets[i,3]}")
            # Throw away EC
            tlvData = tlvData[targetSize:]
        
        return numDetectedTargets, targets

    def rescale_and_emit_points(self, targets):
        """Rescale radar XY coordinates to grid dimensions and emit for plotting."""
        if targets.shape[0] == 0:
            return

        # Confidence filter (keep targets with confidence >= 0.5)
        targets = targets[targets[:, 11] >= 0.5]
        if targets.shape[0] == 0:
            return

        # Check if grid dimensions are set
        if self.nx == 0 or self.ny == 0:
            # Silently skip if grid not initialized (expected during startup)
            return

        rescaled_points = []
        
        for i_t in range(len(targets)):
            target_id = int(targets[i_t, 0])
            
            # Real-world coordinates (meters)
            x_m = targets[i_t, 1]
            y_m = targets[i_t, 2]

            # Rescale to grid dimensions
            # Map x_m from radar range to [x_min, x_max]
            # Map y_m from radar range to [y_min, y_max]
            x_grid = ((x_m - self.x_min) / (self.x_max - self.x_min)) * (self.nx - 1)
            y_grid = ((y_m - self.y_min) / (self.y_max - self.y_min)) * (self.ny - 1)   
            # Clamp to grid bounds
            x_grid = max(0, min(self.nx - 1, x_grid))
            y_grid = max(0, min(self.ny - 1, y_grid))
            print(f"x_m {x_m} y_m {y_m} x_grid {x_grid} y_grid {y_grid} \n")
            
            rescaled_points.append({
                'x': x_grid,
                'y': y_grid,
                'id': target_id,
                'x_real': x_m,
                'y_real': y_m
            })

        # Emit rescaled points to frontend
        self.radar_points_ready.emit(rescaled_points)

    @staticmethod
    def tlv_header_decode(data):
        """Decode TLV header."""
        tlvType, tlvLength = struct.unpack('2I', data)
        return tlvType, tlvLength