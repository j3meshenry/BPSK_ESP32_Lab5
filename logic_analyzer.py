"""
CECS 460 Lab 5 - Logic Analyzer Visualization 
Python script to capture and display waveforms from ESP32

Requirements: pip install pyserial matplotlib numpy
"""

import serial
import serial.tools.list_ports
import time
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

# --- Configuration (MUST MATCH ESP32 CODE) ---
COM_PORT = 'COM3'  # <<<--- CHANGE THIS to ESP32's COM port
BAUD_RATE = 115200
SAMPLES_PER_CAPTURE = 2000
SAMPLE_RATE_HZ = 1000000  # 1 MHz sampling rate

# GPIO Pin numbers (for display only)
TX_PIN = 18 
RX_PIN = 19

# Markers (Must match ESP32 firmware)
START_MARKER = ord('S')  # ASCII value of 'S'
END_MARKER = ord('E')    # ASCII value of 'E'

def list_serial_ports():
    """List all available serial ports"""
    ports = serial.tools.list_ports.comports()
    available_ports = []
    
    print("\nAvailable serial ports:")
    if not ports:
        print("  No serial ports found!")
    else:
        for port in ports:
            print(f"  {port.device} - {port.description}")
            available_ports.append(port.device)
    
    return available_ports

def connect_to_esp32(port, baud):
    """Establish serial connection to ESP32"""
    try:
        print(f"\nAttempting to connect to {port} at {baud} baud...")
        
        # Timeout must be long enough to receive all data
        # 2000 bytes @ 115200 baud = ~174ms + margin
        ser = serial.Serial(port, baud, timeout=0.5)
        
        # Wait for ESP32 to reset after serial connection
        time.sleep(2)
        
        # Synchronize with ESP32 (clear any partial data)
        print("Synchronizing with ESP32...")
        ser.flushInput()
        time.sleep(0.1)
        
        # Discard any pending data
        while ser.in_waiting > 0:
            ser.read(ser.in_waiting)
            time.sleep(0.05)
        
        print("✓ Connection successful!")
        print("✓ Synchronized with ESP32")
        print("\nWaiting for ESP32 to trigger and send data...")
        print("(Trigger occurs on falling edge of TX pin)\n")
        
        return ser
        
    except serial.SerialException as e:
        print(f"\n✗ ERROR: Could not open serial port {port}")
        print("  Please check:")
        print("  - Port name is correct")
        print("  - ESP32 is connected via USB")
        print("  - Port is not already in use (close Arduino IDE serial monitor)")
        print(f"\n  Details: {e}")
        return None

def decode_capture(raw_data):
    """Decode packed binary data into TX and RX signals"""
    # Convert raw bytes to NumPy array
    data_array = np.frombuffer(raw_data, dtype=np.uint8)
    
    # Extract TX signal (bit 0)
    tx_data = data_array & 0x01
    
    # Extract RX signal (bit 1)
    rx_data = (data_array >> 1) & 0x01
    
    return tx_data, rx_data

def analyze_signal(tx_data, rx_data, sample_rate):
    """Analyze captured signals and print statistics"""
    # Count signal transitions (edges)
    tx_transitions = np.sum(np.abs(np.diff(tx_data)))
    rx_transitions = np.sum(np.abs(np.diff(rx_data)))
    
    # Calculate signal statistics
    tx_high_time = np.sum(tx_data) / sample_rate * 1000  # ms
    rx_high_time = np.sum(rx_data) / sample_rate * 1000  # ms
    
    # Find first transition in each signal (for propagation delay)
    tx_first_edge = np.where(np.diff(tx_data) != 0)[0]
    rx_first_edge = np.where(np.diff(rx_data) != 0)[0]
    
    propagation_delay = 0
    if len(tx_first_edge) > 0 and len(rx_first_edge) > 0:
        propagation_delay = (rx_first_edge[0] - tx_first_edge[0]) / sample_rate * 1e6  # μs
    
    # Print analysis
    print("\n" + "="*50)
    print("SIGNAL ANALYSIS")
    print("="*50)
    print(f"TX Signal:")
    print(f"  Transitions (edges): {tx_transitions}")
    print(f"  High time: {tx_high_time:.3f} ms")
    print(f"  Activity: {tx_transitions > 0 and '✓ Active' or '✗ No activity'}")
    
    print(f"\nRX Signal:")
    print(f"  Transitions (edges): {rx_transitions}")
    print(f"  High time: {rx_high_time:.3f} ms")
    print(f"  Activity: {rx_transitions > 0 and '✓ Active' or '✗ No activity'}")
    
    if len(tx_first_edge) > 0 and len(rx_first_edge) > 0:
        print(f"\nPropagation Delay: {propagation_delay:.2f} μs")
    
    # Estimate bit rate from transitions
    if tx_transitions > 2:
        capture_duration = len(tx_data) / sample_rate  # seconds
        estimated_bit_rate = (tx_transitions / 2) / capture_duration  # approximate
        print(f"Estimated Bit Rate: {estimated_bit_rate/1000:.1f} kbps")
    
    print("="*50 + "\n")
    
    # Warnings
    if tx_transitions == 0 and rx_transitions == 0:
        print("⚠ WARNING: No signal activity detected!")
        print("  Check the wiring and ensure transmitter is running.\n")

def save_capture(time_ms, tx_data, rx_data):
    """Save captured data to CSV file"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"logic_capture_{timestamp}.csv"
    
    # Create data matrix
    data_matrix = np.column_stack((time_ms, tx_data, rx_data))
    
    # Save with header
    np.savetxt(filename, data_matrix, delimiter=',', 
               header='Time_ms,TX,RX', comments='', fmt='%.6f,%d,%d')
    
    print(f"✓ Data saved to: {filename}\n")
    return filename

def plot_signals(time_ms, tx_data, rx_data, fig, ax_tx, ax_rx):
    """Plot TX and RX signals"""
    # Clear previous plots
    ax_tx.clear()
    ax_rx.clear()
    
    # Plot as step functions (digital signals)
    ax_tx.step(time_ms, tx_data, where='post', linewidth=1.5, color='red', label='TX')
    ax_rx.step(time_ms, rx_data, where='post', linewidth=1.5, color='blue', label='RX')
    
    # Configure TX axis
    ax_tx.set_title(f'TX Signal (GPIO {TX_PIN})', fontsize=12, fontweight='bold')
    ax_tx.set_ylabel('Logic Level', fontsize=10)
    ax_tx.set_ylim(-0.1, 1.2)
    ax_tx.set_yticks([0, 1])
    ax_tx.set_yticklabels(['LOW', 'HIGH'])
    ax_tx.grid(True, alpha=0.3)
    ax_tx.legend(loc='upper right')
    
    # Configure RX axis
    ax_rx.set_title(f'RX Signal (GPIO {RX_PIN})', fontsize=12, fontweight='bold')
    ax_rx.set_xlabel('Time (ms)', fontsize=10)
    ax_rx.set_ylabel('Logic Level', fontsize=10)
    ax_rx.set_ylim(-0.1, 1.2)
    ax_rx.set_yticks([0, 1])
    ax_rx.set_yticklabels(['LOW', 'HIGH'])
    ax_rx.grid(True, alpha=0.3)
    ax_rx.legend(loc='upper right')
    
    # Update display
    fig.tight_layout()
    fig.canvas.draw()
    fig.canvas.flush_events()

def run_logic_analyzer():
    """Main logic analyzer loop"""
    
    # Show available ports
    available_ports = list_serial_ports()
    
    if not available_ports:
        print("\n✗ No serial ports detected. Please connect the ESP32.")
        return
    
    # Connect to ESP32
    ser = connect_to_esp32(COM_PORT, BAUD_RATE)
    if ser is None:
        return
    
    # Setup matplotlib for live plotting
    plt.ion()  # Interactive mode
    fig, (ax_tx, ax_rx) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    fig.suptitle('ESP32 Logic Analyzer - CECS 460 Lab 5', fontsize=14, fontweight='bold')
    plt.show()
    
    # Calculate time array
    time_us = np.arange(SAMPLES_PER_CAPTURE) * (1e6 / SAMPLE_RATE_HZ)  # Microseconds
    time_ms = time_us / 1000  # Convert to milliseconds
    
    capture_count = 0
    
    print("="*50)
    print("READY TO CAPTURE")
    print("="*50)
    print("Press Ctrl+C to exit\n")
    
    try:
        while True:
            # Read one byte at a time looking for start marker
            if ser.in_waiting > 0:
                byte_data = ser.read(1)
                incoming_byte = byte_data[0]
                
                # Check for start marker
                if incoming_byte == START_MARKER:
                    capture_count += 1
                    print(f"\n[Capture #{capture_count}] Start marker received - Reading data...")
                    
                    # Read the data block
                    raw_data = ser.read(SAMPLES_PER_CAPTURE)
                    
                    # Read end marker
                    end_marker_byte = ser.read(1)
                    
                    # Validate reception
                    if not end_marker_byte or end_marker_byte[0] != END_MARKER:
                        print("✗ ERROR: End marker 'E' not received. Data may be corrupted.")
                        continue
                    
                    if len(raw_data) != SAMPLES_PER_CAPTURE:
                        print(f"✗ ERROR: Expected {SAMPLES_PER_CAPTURE} bytes, got {len(raw_data)}")
                        continue
                    
                    print(f"✓ Received {len(raw_data)} samples successfully")
                    
                    # Decode the data
                    tx_data, rx_data = decode_capture(raw_data)
                    
                    # Analyze signals
                    analyze_signal(tx_data, rx_data, SAMPLE_RATE_HZ)
                    
                    # Plot the results
                    plot_signals(time_ms, tx_data, rx_data, fig, ax_tx, ax_rx)
                    
                    # Ask user if they want to save
                    save_prompt = input("Save this capture to CSV? (y/n): ").strip().lower()
                    if save_prompt == 'y':
                        save_capture(time_ms, tx_data, rx_data)
                    
                    print("\nWaiting for next trigger...\n")
                
            else:
                time.sleep(0.01)  # Small delay to prevent busy-waiting
                
    except KeyboardInterrupt:
        print("\n\n" + "="*50)
        print("Program terminated by user")
        print("="*50)
        print(f"Total captures: {capture_count}")
        
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        # Cleanup
        if ser and ser.is_open:
            ser.close()
            print("\n✓ Serial connection closed")
        
        plt.ioff()
        plt.show()  # Keep final plot open
        print("\n✓ Plot window will remain open")

if __name__ == "__main__":
    print("\n" + "="*50)
    print("ESP32 LOGIC ANALYZER - CECS 460 LAB 5")
    print("="*50)
    run_logic_analyzer()
