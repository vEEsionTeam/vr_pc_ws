import sys
import paramiko
import subprocess
import os
from PySide2.QtWidgets import QApplication, QMainWindow, QMessageBox
from PySide2.QtCore import QThread, Signal, QTime, QTimer
from ui_veesion import Ui_MainWindow
import time
from concurrent.futures import ThreadPoolExecutor
import signal
import re

class LocalProcessWorker(QThread):
    output = Signal(str)
    error = Signal(str)
    finished = Signal()
    process_started = Signal()
    process_stopped = Signal()

    def __init__(self, command):
        super().__init__()
        self.command = command
        self.process = None
        self.is_running = True
        self.process_name = os.path.basename(command.split()[0])  # Get the script name

    def run(self):
        try:
            # Change to the workspace directory before running the command
            workspace_dir = os.path.expanduser("~/vr_pc_ws")
            os.chdir(workspace_dir)
            
            # Use subprocess.Popen with pipes for both stdout and stderr
            self.process = subprocess.Popen(
                self.command,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                universal_newlines=True,
                preexec_fn=os.setsid  # Create a new process group
            )
            self.process_started.emit()

            # Read output and error streams
            while self.is_running:
                # Read stdout
                stdout_line = self.process.stdout.readline()
                if stdout_line:
                    self.output.emit(stdout_line.strip())
                
                # Read stderr
                stderr_line = self.process.stderr.readline()
                if stderr_line:
                    self.error.emit(stderr_line.strip())
                
                # Check if process has ended
                if self.process.poll() is not None:
                    break
                
                # Small delay to prevent CPU overuse
                time.sleep(0.01)

            # Read any remaining output
            remaining_stdout, remaining_stderr = self.process.communicate()
            if remaining_stdout:
                self.output.emit(remaining_stdout.strip())
            if remaining_stderr:
                self.error.emit(remaining_stderr.strip())

        except Exception as e:
            self.error.emit(f"Error in LocalProcessWorker: {str(e)}")
        finally:
            self.process_stopped.emit()
            self.finished.emit()

    def stop(self):
        self.is_running = False
        if self.process:
            try:
                # Send SIGINT (Ctrl+C) to the process
                if "rviz" in self.process_name:
                    subprocess.run("pkill -2 -f rviz", shell=True)
                elif "gazebo" in self.process_name:
                    subprocess.run("pkill -2 -f gazebo", shell=True)
                elif "server" in self.process_name:
                    # More thorough server stopping sequence
                    subprocess.run("pkill -2 -f tf_server2", shell=True)
                    time.sleep(1)
                    subprocess.run("pkill -f tf_server2", shell=True)
                    time.sleep(1)
                    subprocess.run("pkill -f server.sh", shell=True)
                    time.sleep(1)
                    subprocess.run("ps aux | grep -i tf_server2 | grep -v grep | awk '{print $2}' | xargs -r kill -9", shell=True)
                    subprocess.run("ps aux | grep -i server.sh | grep -v grep | awk '{print $2}' | xargs -r kill -9", shell=True)
                
                # Give it a moment to terminate gracefully
                time.sleep(1)
                
                # If process is still running, try to kill it
                if self.process.poll() is None:
                    self.process.terminate()
                    try:
                        self.process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        self.process.kill()
            except:
                pass

class SSHConnection:
    def __init__(self, host, username, password):
        self.host = host
        self.username = username
        self.password = password
        self.ssh = None
        self.channel = None
        self.connect()

    def connect(self):
        max_retries = 2
        retry_delay = 1
        
        for attempt in range(max_retries):
            try:
                if self.ssh:
                    self.ssh.close()
                
                self.ssh = paramiko.SSHClient()
                self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                
                # Add timeout and banner_timeout
                self.ssh.connect(
                    self.host,
                    username=self.username,
                    password=self.password,
                    timeout=5,
                    banner_timeout=5
                )
                
                # Create a new channel with specific terminal settings
                self.channel = self.ssh.invoke_shell(
                    term='xterm',
                    width=80,
                    height=24
                )
                
                # Wait for the shell to be ready
                time.sleep(0.5)
                
                # Check if channel is still open
                if not self.channel.active:
                    raise Exception("Channel not active after creation")
                
                # Send a test command and wait for response
                self.channel.send("echo 'Connection test'\n")
                time.sleep(0.2)
                
                # Read any initial output
                if self.channel.recv_ready():
                    self.channel.recv(1024)
                
                # Source ROS2 with error checking
                self.channel.send("source /opt/ros/humble/setup.bash && echo 'ROS2 sourced successfully'\n")
                time.sleep(0.5)
                
                # Verify channel is still active
                if not self.channel.active:
                    raise Exception("Channel closed after ROS2 sourcing")
                
                return True
                
            except paramiko.AuthenticationException:
                raise Exception(f"Authentication failed for {self.host}")
            except paramiko.SSHException as e:
                if attempt < max_retries - 1:
                    print(f"SSH connection attempt {attempt + 1} failed: {str(e)}")
                    time.sleep(retry_delay)
                    continue
                raise Exception(f"SSH connection failed after {max_retries} attempts: {str(e)}")
            except Exception as e:
                if attempt < max_retries - 1:
                    print(f"Connection attempt {attempt + 1} failed: {str(e)}")
                    time.sleep(retry_delay)
                    continue
                raise Exception(f"Connection failed after {max_retries} attempts: {str(e)}")
        
        return False

    def send_command(self, command):
        if not self.channel or not self.channel.active:
            print("Channel not active, attempting to reconnect...")
            if not self.connect():
                return False
            
        try:
            # For multi-line commands, split and send each line
            if isinstance(command, str) and '\n' in command:
                for line in command.strip().split('\n'):
                    if line.strip():  # Only send non-empty lines
                        self.channel.send(line.strip() + "\n")
                        time.sleep(0.1)  # Small delay between commands
            else:
                self.channel.send(command + "\n")
            return True
        except Exception as e:
            print(f"Error sending command: {str(e)}")
            return False

    def read_output(self):
        if not self.channel or not self.channel.active:
            return None
        
        try:
            if self.channel.recv_ready():
                return self.channel.recv(1024).decode('utf-8')
            return None
        except:
            return None

    def close(self):
        if self.ssh:
            try:
                if self.channel:
                    self.channel.close()
                self.ssh.close()
            except:
                pass

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        # Set default IP addresses
        self.ui.textEdit.setPlainText("192.168.1.2")  # Default RPI IP
        self.ui.textEdit_2.setPlainText("192.168.1.4")  # Default server IP
        
        # Initialize SSH connections dictionary
        self.ssh_connections = {}
        self.stop_connection = None  # Dedicated connection for stop commands
        
        # Initialize processes
        self.server_process = None
        self.rviz_process = None
        self.gazebo_process = None
        
        # Initialize output timers
        self.output_timers = {}
        
        # Connect signals
        self.ui.pushButton_connect.clicked.connect(self.connect_to_rpi)
        self.ui.pb_run_cam.clicked.connect(lambda: self.run_command("camera"))
        self.ui.pb_stop_cam.clicked.connect(lambda: self.stop_command("camera"))
        self.ui.pb_run_imu.clicked.connect(lambda: self.run_command("imu"))
        self.ui.pb_stop_imu.clicked.connect(lambda: self.stop_command("imu"))
        self.ui.pb_run_client.clicked.connect(lambda: self.run_command("client"))
        self.ui.pb_stop_client.clicked.connect(lambda: self.stop_command("client"))
        self.ui.pb_run_play.clicked.connect(lambda: self.run_command("play"))
        self.ui.pb_stop_play.clicked.connect(lambda: self.stop_command("play"))
        
        # Connect server control signals
        self.ui.pb_run_server.clicked.connect(self.run_server)
        self.ui.pb_stop_server.clicked.connect(self.stop_server)
        
        # Connect RViz and Gazebo control signals
        self.ui.pb_run_rviz.clicked.connect(self.run_rviz)
        self.ui.pb_stop_rviz.clicked.connect(self.stop_rviz)
        self.ui.pb_run_gazebo.clicked.connect(self.run_gazebo)
        self.ui.pb_stop_gazebo.clicked.connect(self.stop_gazebo)
        
        # Initialize status
        self.update_status("Disconnected")
        self.disable_buttons()
        
        # Initially disable stop buttons for RViz and Gazebo
        self.ui.pb_stop_rviz.setEnabled(False)
        self.ui.pb_stop_gazebo.setEnabled(False)

    def connect_to_rpi(self):
        ip = self.ui.textEdit.toPlainText().strip()
        if not ip:
            QMessageBox.warning(self, "Error", "Please enter RPI IP address")
            return

        try:
            # Create all connections in parallel using threads
            
            def create_connection(component):
                try:
                    connection = SSHConnection(ip, "veesion", "1234")
                    if connection.connect():
                        return component, connection
                    return component, None
                except Exception as e:
                    print(f"Failed to connect {component}: {str(e)}")
                    return component, None
            
            # Create connections in parallel
            components = ["camera", "imu", "client", "play", "stop"]  # Added stop connection
            with ThreadPoolExecutor(max_workers=5) as executor:
                results = list(executor.map(create_connection, components))
            
            # Process results
            for component, connection in results:
                if connection:
                    if component == "stop":
                        self.stop_connection = connection
                    else:
                        self.ssh_connections[component] = connection
                    print(f"{component} terminal connected successfully")
                else:
                    raise Exception(f"Failed to connect {component} terminal")
            
            self.update_status("Connected")
            self.enable_buttons()
            self.append_to_log("Connected to RPI at " + ip, "System")
            
        except Exception as e:
            error_msg = f"Failed to connect to RPI: {str(e)}"
            print(error_msg)
            QMessageBox.critical(self, "Error", error_msg)
            self.update_status("Connection Failed")
            self.disable_buttons()
            # Close any existing connections
            for conn in self.ssh_connections.values():
                conn.close()
            if self.stop_connection:
                self.stop_connection.close()
            self.ssh_connections.clear()
            self.stop_connection = None

    def run_command(self, command_type):
        if command_type not in self.ssh_connections:
            self.append_to_log(f"No connection available for {command_type}", "System")
            return

        commands = {
            "camera": "./vr_scripts/camera.sh",
            "imu": "./vr_scripts/imu.sh",
            "client": self.get_client_command(),
            "play": "./vr_scripts/openvins.sh"  # Simply run the script
        }

        # First stop any existing process
        self.stop_command(command_type)

        # For client, check if server is running
        if command_type == "client" and not self.is_server_running():
            QMessageBox.warning(self, "Error", "Start server first!")
            return

        # Send the command to the appropriate terminal
        if self.ssh_connections[command_type].send_command(commands[command_type]):
            self.append_to_log(f"Started {command_type}", "System")
            self.on_process_started(command_type)
            
            # Start output timer for this command
            self.start_output_timer(command_type)
            
            # Clear the appropriate text browser before starting
            if command_type == "play":
                self.ui.textBrowser.clear()  # Clear play's textBrowser
        else:
            self.append_to_log(f"Failed to start {command_type}", "System")

    def get_client_command(self):
        # Get parameters from UI
        img_param = "1" if self.ui.checkBox_img.isChecked() else "0"
        path_param = "1" if self.ui.checkBox_path.isChecked() else "0"
        points_param = "1" if self.ui.checkBox_points.isChecked() else "0"
        ip_param = self.ui.textEdit_2.toPlainText().strip()
        
        if not ip_param:
            QMessageBox.warning(self, "Error", "Please enter IP address for server")
            return None

        # Construct the client command with parameters
        return f"./vr_scripts/tf_client.py --ros-args -p img_enable:={img_param} -p path_enable:={path_param} -p points_enable:={points_param} -p server_ip:={ip_param}"

    def is_server_running(self):
        if not self.server_process:
            return False
        return self.server_process.isRunning()

    def stop_command(self, command_type):
        if command_type not in self.ssh_connections:
            return

        # Stop the output timer
        self.stop_output_timer(command_type)

        stop_commands = {
            "camera": """
                pkill -f camera.sh
                pkill -f camera
                pkill -f ros2.*camera
                sleep 1
                ps aux | grep -i camera | grep -v grep | awk '{print $2}' | xargs -r kill -9
            """,
            "imu": """
                pkill -f bno055
                pkill -2 -f bno055
                pkill -f imu.sh
                pkill -f ros2.*imu
                sleep 1
                ps aux | grep -i bno055 | grep -v grep | awk '{print $2}' | xargs -r kill -9
            """,
            "client": """
                pkill -f client.sh
                pkill -f tf_client.py
                pkill -f ros2.*client
                sleep 1
                ps aux | grep -i tf_client | grep -v grep | awk '{print $2}' | xargs -r kill -9
            """,
            "play": """
                pkill -2 -f ov_msckf
                sleep 1
                pkill -f ov_msckf
                sleep 2
                pkill -f ov_msckf
                sleep 1
                ps aux | grep -i ov_msckf | grep -v grep | awk '{print $2}' | xargs -r kill -9
                pkill -f openvins_node
                pkill -f ros2.*openvins
                sleep 1
                ps aux | grep -i openvins | grep -v grep | awk '{print $2}' | xargs -r kill -9
            """
        }

        try:
            # Create a new SSH connection for stop command
            ip = self.ui.textEdit.toPlainText().strip()
            stop_connection = SSHConnection(ip, "veesion", "1234")
            
            if stop_connection.connect():
                # Send the stop command
                if stop_connection.send_command(stop_commands[command_type]):
                    self.append_to_log(f"Sent stop command for {command_type}", "System")
                    
                    # Wait a moment for processes to stop
                    time.sleep(2)
                    
                    # Verify if processes are still running
                    verify_command = f"ps aux | grep -i {command_type} | grep -v grep"
                    if stop_connection.send_command(verify_command):
                        self.append_to_log(f"Verifying {command_type} processes stopped", "System")
                    
                    self.on_process_stopped(command_type)
                else:
                    self.append_to_log(f"Failed to send stop command for {command_type}", "System")
                
                # Close the stop connection
                stop_connection.close()
            else:
                self.append_to_log(f"Failed to create stop connection for {command_type}", "System")
                
        except Exception as e:
            self.append_to_log(f"Error stopping {command_type}: {str(e)}", "System")

    def on_process_started(self, command_type):
        if command_type == "camera":
            self.ui.pb_run_cam.setEnabled(False)
            self.ui.pb_stop_cam.setEnabled(True)
        elif command_type == "imu":
            self.ui.pb_run_imu.setEnabled(False)
            self.ui.pb_stop_imu.setEnabled(True)
        elif command_type == "client":
            self.ui.pb_run_client.setEnabled(False)
            self.ui.pb_stop_client.setEnabled(True)
        elif command_type == "play":
            self.ui.pb_run_play.setEnabled(False)
            self.ui.pb_stop_play.setEnabled(True)
            self.append_to_log("OpenVINS started", "System")

    def on_process_stopped(self, command_type):
        if command_type == "camera":
            self.ui.pb_run_cam.setEnabled(True)
            self.ui.pb_stop_cam.setEnabled(False)
        elif command_type == "imu":
            self.ui.pb_run_imu.setEnabled(True)
            self.ui.pb_stop_imu.setEnabled(False)
        elif command_type == "client":
            self.ui.pb_run_client.setEnabled(True)
            self.ui.pb_stop_client.setEnabled(False)
        elif command_type == "play":
            self.ui.pb_run_play.setEnabled(True)
            self.ui.pb_stop_play.setEnabled(False)
            self.append_to_log("OpenVINS stopped", "System")

    def update_status(self, status):
        self.ui.textBrowser_2.setText(status)

    def append_to_log(self, message, source):
        timestamp = QTime.currentTime().toString("hh:mm:ss")
        log_entry = f"[{timestamp}] {source}: {message}"
        
        # Add to main log
        self.ui.textBrowser.append(log_entry)
        
        # Add to specific tabs
        if source == "Camera":
            self.ui.textBrowser_4.append(log_entry)
        elif source == "IMU":
            self.ui.textBrowser_5.append(message)  # Don't add timestamp for IMU output
        elif source == "Client":
            self.ui.textBrowser_7.append(log_entry)

    def enable_buttons(self):
        self.ui.pb_run_cam.setEnabled(True)
        self.ui.pb_run_imu.setEnabled(True)
        self.ui.pb_run_client.setEnabled(True)
        self.ui.pb_run_play.setEnabled(True)

    def disable_buttons(self):
        self.ui.pb_run_cam.setEnabled(False)
        self.ui.pb_stop_cam.setEnabled(False)
        self.ui.pb_run_imu.setEnabled(False)
        self.ui.pb_stop_imu.setEnabled(False)
        self.ui.pb_run_client.setEnabled(False)
        self.ui.pb_stop_client.setEnabled(False)
        self.ui.pb_run_play.setEnabled(False)
        self.ui.pb_stop_play.setEnabled(False)

    def closeEvent(self, event):
        # Stop all running commands
        for worker in self.ssh_connections.values():
            worker.close()
        
        # Stop server if running
        if self.server_process:
            try:
                self.server_process.stop()
                if self.server_process.isRunning():
                    self.server_process.wait()
                self.server_process.deleteLater()
            except:
                pass
        
        # Stop RViz if running
        if self.rviz_process:
            try:
                self.rviz_process.stop()
                if self.rviz_process.isRunning():
                    self.rviz_process.wait()
                self.rviz_process.deleteLater()
            except:
                pass
        
        # Stop Gazebo if running
        if self.gazebo_process:
            try:
                self.gazebo_process.stop()
                if self.gazebo_process.isRunning():
                    self.gazebo_process.wait()
                self.gazebo_process.deleteLater()
            except:
                pass
        
        event.accept()

    def stop_server(self):
        if self.server_process:
            try:
                self.server_process.stop()
                # Wait for the process to stop
                time.sleep(2)
                # Update UI state
                self.on_server_stopped()
                # Clean up the thread
                if self.server_process.isRunning():
                    self.server_process.wait()
                self.server_process = None
            except Exception as e:
                self.append_to_log(f"Error stopping server: {str(e)}", "System")

    def on_server_started(self):
        self.ui.pb_run_server.setEnabled(False)
        self.ui.pb_stop_server.setEnabled(True)
        self.append_to_log("Server started", "System")
        self.ui.textBrowser_3.setText("Open")

    def on_server_stopped(self):
        self.ui.pb_run_server.setEnabled(True)
        self.ui.pb_stop_server.setEnabled(False)
        self.append_to_log("Server stopped", "System")
        self.ui.textBrowser_3.setText("Closed")

    def on_server_finished(self):
        if self.server_process:
            self.server_process.deleteLater()
            self.server_process = None

    def run_rviz(self):
        # Stop any existing RViz process
        self.stop_rviz()
        
        # Create and start new RViz process
        self.rviz_process = LocalProcessWorker("./pc_scripts/rviz.sh")
        self.rviz_process.output.connect(lambda msg: self.append_to_log(msg, "RViz"))
        self.rviz_process.error.connect(lambda msg: self.append_to_log(f"Error: {msg}", "RViz"))
        self.rviz_process.process_started.connect(self.on_rviz_started)
        self.rviz_process.process_stopped.connect(self.on_rviz_stopped)
        self.rviz_process.finished.connect(self.on_rviz_finished)
        
        self.rviz_process.start()
        self.append_to_log("Starting RViz...", "System")

    def stop_rviz(self):
        if self.rviz_process:
            try:
                self.rviz_process.stop()
                # Wait for the process to stop
                time.sleep(2)
                # Update UI state
                self.on_rviz_stopped()
                # Clean up the thread
                if self.rviz_process.isRunning():
                    self.rviz_process.wait()
                self.rviz_process = None
            except Exception as e:
                self.append_to_log(f"Error stopping RViz: {str(e)}", "System")

    def on_rviz_started(self):
        self.ui.pb_run_rviz.setEnabled(False)
        self.ui.pb_stop_rviz.setEnabled(True)
        self.append_to_log("RViz started", "System")

    def on_rviz_stopped(self):
        self.ui.pb_run_rviz.setEnabled(True)
        self.ui.pb_stop_rviz.setEnabled(False)
        self.append_to_log("RViz stopped", "System")

    def on_rviz_finished(self):
        if self.rviz_process:
            self.rviz_process.deleteLater()
            self.rviz_process = None

    def run_gazebo(self):
        # Stop any existing Gazebo process
        self.stop_gazebo()
        
        # Get selected world from comboBox
        selected_world = self.ui.comboBox_worlds.currentText()
        if not selected_world:
            QMessageBox.warning(self, "Error", "Please select a world from the dropdown")
            return
        
        # Create and start new Gazebo process with selected world
        command = f"cd ~/vr_pc_ws && source install/setup.bash && ros2 launch gazebo_sim gazebo.launch.py world:={selected_world}"
        self.gazebo_process = LocalProcessWorker(command)
        self.gazebo_process.output.connect(lambda msg: self.append_to_log(msg, "Gazebo"))
        self.gazebo_process.error.connect(lambda msg: self.append_to_log(f"Error: {msg}", "Gazebo"))
        self.gazebo_process.process_started.connect(self.on_gazebo_started)
        self.gazebo_process.process_stopped.connect(self.on_gazebo_stopped)
        self.gazebo_process.finished.connect(self.on_gazebo_finished)
        
        self.gazebo_process.start()
        self.append_to_log(f"Starting Gazebo with world: {selected_world}", "System")

    def stop_gazebo(self):
        if self.gazebo_process:
            try:
                self.gazebo_process.stop()
                # Wait for the process to stop
                time.sleep(2)
                # Update UI state
                self.on_gazebo_stopped()
                # Clean up the thread
                if self.gazebo_process.isRunning():
                    self.gazebo_process.wait()
                self.gazebo_process = None
            except Exception as e:
                self.append_to_log(f"Error stopping Gazebo: {str(e)}", "System")

    def on_gazebo_started(self):
        self.ui.pb_run_gazebo.setEnabled(False)
        self.ui.pb_stop_gazebo.setEnabled(True)
        self.append_to_log("Gazebo started", "System")

    def on_gazebo_stopped(self):
        self.ui.pb_run_gazebo.setEnabled(True)
        self.ui.pb_stop_gazebo.setEnabled(False)
        self.append_to_log("Gazebo stopped", "System")

    def on_gazebo_finished(self):
        if self.gazebo_process:
            self.gazebo_process.deleteLater()
            self.gazebo_process = None

    def start_output_timer(self, command_type):
        # Stop existing timer if any
        if command_type in self.output_timers:
            self.output_timers[command_type].stop()
        
        # Create new timer
        timer = QTimer()
        timer.timeout.connect(lambda: self.read_command_output(command_type))
        timer.start(100)  # Read every 100ms
        self.output_timers[command_type] = timer

    def stop_output_timer(self, command_type):
        if command_type in self.output_timers:
            self.output_timers[command_type].stop()
            del self.output_timers[command_type]

    def read_command_output(self, command_type):
        if command_type not in self.ssh_connections:
            return
        
        output = self.ssh_connections[command_type].read_output()
        if output:
            # Clear any ANSI escape sequences
            output = re.sub(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])', '', output)
            
            # Display in appropriate text browser
            if command_type == "camera":
                self.ui.textBrowser_4.append(output.strip())
            elif command_type == "imu":
                self.ui.textBrowser_5.append(output.strip())
            elif command_type == "client":
                self.ui.textBrowser_7.append(output.strip())
            elif command_type == "play":
                self.ui.textBrowser.append(output.strip())  # Add output to play's textBrowser

    def run_server(self):
        # Get parameters from UI
        img_param = "1" if self.ui.checkBox_img.isChecked() else "0"
        path_param = "1" if self.ui.checkBox_path.isChecked() else "0"
        points_param = "1" if self.ui.checkBox_points.isChecked() else "0"
        ip_param = self.ui.textEdit_2.toPlainText().strip()
        
        if not ip_param:
            QMessageBox.warning(self, "Error", "Please enter IP address for server")
            return

        # Construct the command
        command = f"./pc_scripts/server.sh --img {img_param} --path {path_param} --points {points_param} --ip {ip_param}"
        
        # Stop any existing server process
        self.stop_server()
        
        # Create and start new server process
        self.server_process = LocalProcessWorker(command)
        
        # Connect output signals to textBrowser_6
        self.server_process.output.connect(lambda msg: self.ui.textBrowser_6.append(msg))
        self.server_process.error.connect(lambda msg: self.ui.textBrowser_6.append(f"Error: {msg}"))
        
        # Connect other signals
        self.server_process.process_started.connect(self.on_server_started)
        self.server_process.process_stopped.connect(self.on_server_stopped)
        self.server_process.finished.connect(self.on_server_finished)
        
        # Start the process
        self.server_process.start()
        self.append_to_log("Starting server...", "System")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_()) 