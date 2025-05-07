import tkinter as tk
from tkinter import ttk, messagebox, Listbox, filedialog
import subprocess
import threading
import geocoder
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.path as mpath
import matplotlib.patches as mpatches
from mpl_toolkits.basemap import Basemap
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import time
import logging
import os
import re
from PIL import Image, ImageOps, ImageTk  # Remove ImageResampling from imports

matplotlib.use('TkAgg')
plt.rcParams['path.simplify'] = True
plt.rcParams['path.simplify_threshold'] = 0.1
DEFAULT_ADB_PORT = "5555"
LOG_FILE = "adb_controller.log"
REFRESH_INTERVAL = 5000  # 5 seconds

# Setup logging
logging.basicConfig(
    filename=LOG_FILE,
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# ─── Global Variables ─────────────────────────────────────────────────────
all_ips = []                   # Stores IPs with ports (ip:port)
country_ip_map = {}            # country -> list of ips
country_coords = {}            # country -> (lat, lng)
country_patches = {}           # country -> patch data
selected_ip = [None]           # Currently selected IP
scrcpy_process = None          # scrcpy process handle
current_scale = 1.0            # Map zoom scale
connected_ips = set()          # Verified connected IPs
connection_attempts = {}       # IP -> attempt count
pan_start = [None, None]       # Panning coordinates
is_dragging = False            # Panning state
render_timer = None            # Map update timer
connection_lock = threading.Lock()
status_update = None
ip_coords = {}

# ─── Performance Constants ────────────────────────────────────────────────
SIMPLIFY_TOLERANCE = 0.03      
DEBOUNCE_DELAY = 150           
MIN_PATCH_SIZE = 5             


COUNTRY_NAME_MAP = {
    "korea, republic of": "kr",
    "united states": "us",
    "hong kong": "hk",
    # Add more mappings as needed
}

def normalize_country_name(name):
    return COUNTRY_NAME_MAP.get(name.lower(), name.lower())

# ─── ADB Command Handlers ─────────────────────────────────────────────────
def get_adb_devices():
    """Get current connected devices with proper parsing"""
    try:
        result = subprocess.run(
            ["adb", "devices"],
            capture_output=True,
            text=True,
            timeout=10
        )
        devices = []
        # Improved regex parsing
        pattern = re.compile(r'^([^\s]+)\s+([^\s]+)$')
        for line in result.stdout.splitlines():
            match = pattern.match(line)
            if match and match.group(2) in ['device', 'offline']:
                devices.append(match.group(1))
        return devices
    except Exception as e:
        logging.error(f"Failed to get ADB devices: {str(e)}")
        return []

def run_adb_command(command):
    ip = selected_ip[0]
    if not ip:
        messagebox.showwarning("No IP Selected", "Select an IP from the list.")
        return None

    debug_info = {"steps": [], "errors": [], "raw_output": "", "devices": ""}
    
    try:
        with connection_lock:
            # Get initial state
            debug_info["devices"] = "\n".join(get_adb_devices())
            
            # Check connection state
            connected_devices = get_adb_devices()
            if ip not in connected_devices:
                debug_info["steps"].append(f"Connecting to {ip}...")
                connect_result = subprocess.run(
                    ["adb", "connect", ip],
                    capture_output=True,
                    text=True,
                    timeout=20
                )
                debug_info["raw_output"] += f"Connect output:\n{connect_result.stdout}\n"
                
                if "unable" in connect_result.stdout.lower():
                    debug_info["errors"].append("Connection failed")
                    return debug_info

            # Execute command
            debug_info["steps"].append(f"Executing: {command}")
            result = subprocess.run(
                command.split(),
                capture_output=True,
                text=True,
                timeout=25
            )
            debug_info["raw_output"] += f"Command output:\n{result.stdout}\n"
            
            # Update connection status
            current_devices = get_adb_devices()
            if ip in current_devices:
                connected_ips.add(ip)
            else:
                connected_ips.discard(ip)
            
    except Exception as e:
        debug_info["errors"].append(f"Error: {str(e)}")
        logging.error(f"Command error: {str(e)}")
    finally:
        update_map_display()
        schedule_status_update()
        return debug_info

def refresh_connections():
    """Refresh connections by attempting to connect to each IP."""
    try:
        with connection_lock:
            global connected_ips
            connected_ips.clear()  # Reset connected IPs
            
            for ip in all_ips:
                try:
                    result = subprocess.run(
                        ["adb", "connect", ip],
                        capture_output=True,
                        text=True,
                        timeout=10
                    )
                    if "connected" in result.stdout.lower() or "already connected" in result.stdout.lower():
                        connected_ips.add(ip)
                        logging.info(f"Successfully connected to {ip}")
                    else:
                        logging.warning(f"Failed to connect to {ip}: {result.stdout.strip()}")
                except subprocess.TimeoutExpired:
                    logging.error(f"Connection attempt to {ip} timed out.")
                except Exception as e:
                    logging.error(f"Error connecting to {ip}: {str(e)}")
            
            logging.debug(f"Final connected_ips: {connected_ips}")
            
            # Update the listbox, map display, and connection stats
            _reset_overlay_list()
            update_map_display()
            update_connection_stats()  # Update stats here
            
            # Notify the user of the refresh status
            messagebox.showinfo("Refresh Complete", f"Connected to {len(connected_ips)} devices.")
    except Exception as e:
        messagebox.showerror("Refresh Error", str(e))
        logging.error(f"Refresh error: {str(e)}")

def schedule_status_update():
    global status_update
    if status_update:
        root.after_cancel(status_update)
    status_update = root.after(REFRESH_INTERVAL, refresh_connections)

def threaded_command(cmd):
    t = threading.Thread(target=run_adb_command, args=(cmd,))
    t.daemon = True
    t.start()

# ─── Diagnostic Tools ─────────────────────────────────────────────────────
def show_debug_info(info):
    report = "=== Diagnostic Report ===\n"
    report += "\n".join([f"• {step}" for step in info["steps"]])
    
    if info["errors"]:
        report += "\n\n=== Errors ===\n"
        report += "\n".join([f"⛔ {err}" for err in info["errors"]])
    
    report += "\n\n=== ADB Devices ===\n"
    report += info.get("devices", "No device info")
    
    report += "\n\n=== Raw Output ===\n"
    report += info["raw_output"]
    
    messagebox.showinfo("Diagnostics", report)

def raw_ping_test():
    ip = selected_ip[0]
    if not ip:
        return
    
    clean_ip = ip.split(':')[0]
    response = os.system(f"ping -c 1 {clean_ip}")
    if response == 0:
        messagebox.showinfo("Ping", f"{clean_ip} responds to ping")
    else:
        messagebox.showerror("Ping", f"No response from {clean_ip}")

def refresh_connections():
    try:
        with connection_lock:
            result = subprocess.run(
                ["adb", "devices"],
                capture_output=True,
                text=True,
                timeout=15
            )
            connected = []
            for line in result.stdout.splitlines():
                if any(state in line for state in ['device', 'offline']):
                    parts = line.strip().split()
                    if parts and 'List' not in line:
                        connected.append(parts[0])
            global connected_ips
            connected_ips = {ip for ip in all_ips if ip in connected}
            logging.debug(f"Updated connected_ips: {connected_ips}")
            update_map_display()
    except Exception as e:
        messagebox.showerror("Refresh Error", str(e))
        logging.error(f"Refresh error: {str(e)}")

def debug_connection():
    ip = selected_ip[0]
    if not ip:
        messagebox.showwarning("No IP Selected", "Select an IP first")
        return
    
    debug_info = run_adb_command("echo Testing connection")
    if debug_info:
        show_debug_info(debug_info)

def enable_tcp_mode():
    try:
        result = subprocess.run(
            "adb tcpip 5555",
            shell=True,
            capture_output=True,
            text=True,
            timeout=20
        )
        output = result.stdout.lower()
        if "restarting" in output or "5555" in output:
            messagebox.showinfo("TCP Mode", "TCP mode enabled successfully")
        else:
            messagebox.showerror("TCP Mode Error", result.stdout)
    except Exception as e:
        messagebox.showerror("Error", f"Failed to enable TCP mode: {str(e)}")

# ─── IP Management ───────────────────────────────────────────────────────
def normalize_ip(ip):
    """Ensure IP has port number and valid format"""
    ip = ip.strip()
    if not re.match(r'^[\d\.]+:\d+$', ip):
        return f"{ip}:{DEFAULT_ADB_PORT}"
    return ip

def add_ip(ip):
    ip = normalize_ip(ip)
    if ip in all_ips:
        return
    all_ips.append(ip)
    try:
        # Geocode actual IP location (without port)
        g = geocoder.ip(ip.split(':')[0])
        if not g.ok:
            logging.warning(f"Geocoding failed for {ip}")
            return
        # Store country association
        country = g.country.strip().lower()
        country_ip_map.setdefault(country, []).append(ip)
        logging.debug(f"Added IP {ip} to country {country}")
        # Store individual IP coordinates
        if g.latlng:
            ip_coords[ip] = g.latlng  # Store per-IP coordinates
    except Exception as e:
        logging.error(f"Geocoding error for {ip}: {str(e)}")

def load_ips_from_file():
    path = filedialog.askopenfilename(
        title="Select IP List File", 
        filetypes=[("Text Files", "*.txt")]
    )
    if not path:
        return
    # Clear existing data
    all_ips.clear()
    country_ip_map.clear()
    ip_coords.clear()
    ip_listbox.delete(0, tk.END)
    with open(path) as f:
        for line in f:
            raw_ip = line.strip()
            if (raw_ip):
                ip = normalize_ip(raw_ip)
                add_ip(ip)
    # Log the final state of country_ip_map
    logging.debug(f"Loaded country_ip_map: {country_ip_map}")
    # Update listbox
    for ip in all_ips:
        ip_listbox.insert(tk.END, ip)
    update_map_display()

# ─── scrcpy Handlers ──────────────────────────────────────────────────────
def start_scrcpy():
    ip = selected_ip[0]
    if not ip:
        return
    
    try:
        global scrcpy_process
        # Ensure connection before launching
        subprocess.run(["adb", "connect", ip], timeout=10)
        scrcpy_process = subprocess.Popen(
            ["scrcpy", "-s", ip, "--no-audio"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        # Immediate status update
        refresh_connections()
        schedule_status_update()
    except Exception as e:
        messagebox.showerror("scrcpy Error", str(e))

def stop_scrcpy():
    global scrcpy_process
    if scrcpy_process:
        scrcpy_process.terminate()
        scrcpy_process = None
    refresh_connections()

# ─── Map Visualization ────────────────────────────────────────────────────
def create_country_patches():
    for info, shape in zip(m.countries_info, m.countries):
        name = (info.get('ADMIN') or info.get('NAME') or '').lower().strip()
        name = normalize_country_name(name)  # Normalize country name
        proj = [m(lon, lat) for lon, lat in shape]
        try:
            path = mpath.Path(proj).simplify(SIMPLIFY_TOLERANCE)
        except AttributeError:
            path = mpath.Path(proj)
        extents = path.get_extents()
        country_patches[name] = {
            'patch': mpatches.PathPatch(
                path,
                facecolor='dimgray',
                edgecolor='lime',
                alpha=0.3,
                visible=False,
                zorder=1
            ),
            'bbox': (extents.x0, extents.y0, extents.x1, extents.y1)
        }
        ax.add_patch(country_patches[name]['patch'])

def update_map_display():
    global render_timer
    if render_timer:
        root.after_cancel(render_timer)
    render_timer = root.after(DEBOUNCE_DELAY, _perform_map_update)

def _perform_map_update():
    # Get current connected devices
    current_connected = get_adb_devices()
    
    # Prepare data for plotting
    lats, lngs = [], []
    for ip in all_ips:
        if ip in current_connected and ip in ip_coords:
            lat, lng = ip_coords[ip]
            lats.append(lat)
            lngs.append(lng)
    
    # Transform coordinates
    if lats and lngs:
        xpts, ypts = m(lngs, lats)
        points = np.column_stack([xpts, ypts])
        size = max(30 / current_scale, 5)
        
        if hasattr(ax, 'device_points'):
            ax.device_points.set_offsets(points)
            ax.device_points.set_sizes([size] * len(points))
        else:
            ax.device_points = ax.scatter(
                points[:, 0], points[:, 1],
                s=size,
                c='red', edgecolors='lime', linewidths=0.5,
                zorder=2
            )
    elif hasattr(ax, 'device_points'):
        ax.device_points.remove()
        del ax.device_points
    
    # Country highlighting (keep existing code)
    country_connections = {}
    for country, ips in country_ip_map.items():
        count = sum(1 for ip in ips if ip in current_connected)
        if count > 0:
            country_connections[country.lower()] = count
    
    max_connections = max(country_connections.values()) if country_connections else 1
    for name, data in country_patches.items():
        patch = data['patch']
        if name in country_connections:
            intensity = 0.3 + 0.7 * (country_connections[name] / max_connections)
            patch.set_facecolor((intensity, 0, 0))
            patch.set_alpha(0.9)
            patch.set_visible(True)
        else:
            patch.set_visible(False)
    
    canvas.draw_idle()

# ─── Event Handlers ───────────────────────────────────────────────────────
def zoom(event):
    global current_scale
    if event.inaxes != ax or event.button not in ('up', 'down'):
        return
    
    scale_factor = 1.2 if event.button == 'down' else 0.8
    current_scale *= scale_factor
    
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()
    xdata, ydata = event.xdata, event.ydata
    
    ax.set_xlim([
        xdata - (xdata - xlim[0]) * scale_factor,
        xdata + (xlim[1] - xdata) * scale_factor
    ])
    ax.set_ylim([
        ydata - (ydata - ylim[0]) * scale_factor,
        ydata + (ylim[1] - ydata) * scale_factor
    ])
    
    update_map_display()

def on_press(event):
    global is_dragging
    if event.inaxes != ax:
        return
    is_dragging = True
    pan_start[:] = [event.x, event.y]
    ax._start_limits = (ax.get_xlim(), ax.get_ylim())

def on_drag(event):
    global is_dragging
    if not is_dragging or event.inaxes != ax:
        return
    
    inv = ax.transData.inverted()
    start_data = inv.transform(pan_start)
    end_data = inv.transform((event.x, event.y))
    dx = start_data[0] - end_data[0]
    dy = start_data[1] - end_data[1]
    
    xlim, ylim = ax._start_limits
    ax.set_xlim([xlim[0] + dx, xlim[1] + dx])
    ax.set_ylim([ylim[0] + dy, ylim[1] + dy])
    
    update_map_display()

def on_release(event):
    global is_dragging
    is_dragging = False

def hover_handler(event):
    if (event.inaxes != ax or event.xdata is None):
        _reset_overlay_list()
        return
    point = (event.xdata, event.ydata)
    for name, data in country_patches.items():
        patch = data['patch']
        if not patch.get_visible():
            continue
        x0, y0, x1, y1 = data['bbox']
        if not (x0 <= point[0] <= x1 and y0 <= point[1] <= y1):
            continue
        if patch.get_path().contains_point(point):
            country_name = normalize_country_name(name)  # Normalize country name
            logging.debug(f"Hover detected over country: {country_name}")
            _update_hover_display(country_name.lower())  # Ensure lowercase
            return
    _reset_overlay_list()

def _update_hover_display(country_name):
    logging.debug(f"Received country_name: {country_name}")
    
    # Get the IPs associated with the hovered country
    ips = country_ip_map.get(country_name.lower(), [])
    logging.debug(f"IPs found for {country_name}: {ips}")
    
    # Separate connected and disconnected IPs
    connected = [ip for ip in ips if ip in connected_ips]
    disconnected = [ip for ip in ips if ip not in connected_ips]  # Fixed logic
    
    logging.debug(f"Connected IPs: {connected}")
    logging.debug(f"Disconnected IPs: {disconnected}")
    
    # Update the overlay text with the country name and connection stats
    overlay.set_text(f"{country_name.capitalize()}\nConnected: {len(connected)}/{len(ips)}")
    
    # Update the listbox to show only the IPs for the hovered country
    ip_listbox.delete(0, tk.END)
    for ip in connected:
        ip_listbox.insert(tk.END, f"✓ {ip}")
        ip_listbox.itemconfig(tk.END, {'fg': 'lime'})
    for ip in disconnected:
        ip_listbox.insert(tk.END, f"✗ {ip}")
        ip_listbox.itemconfig(tk.END, {'fg': 'red'})
    
    # Redraw the canvas to reflect changes
    canvas.draw_idle()

def test_listbox_update():
    ip_listbox.delete(0, tk.END)
    ip_listbox.insert(tk.END, "✓ 183.112.32.41:5555")
    ip_listbox.itemconfig(tk.END, {'fg': 'lime'})
    ip_listbox.insert(tk.END, "✗ 1.248.4.93:5555")
    ip_listbox.itemconfig(tk.END, {'fg': 'red'})

def _reset_overlay_list():
    """Reset the overlay list and update connection stats."""
    ip_listbox.delete(0, tk.END)
    current_devices = get_adb_devices()  # Real-time check
    for ip in all_ips:
        status = "✓ " if ip in current_devices else "✗ "
        color = 'lime' if ip in current_devices else 'red'
        ip_listbox.insert(tk.END, f"{status}{ip}")
        ip_listbox.itemconfig(tk.END, {'fg': color})
    update_connection_stats()  # Update stats here
    canvas.draw_idle()

# ─── UI Setup ─────────────────────────────────────────────────────────────
root = tk.Tk()
root.title("ADB Global Controller - Diagnostics Edition")
root.geometry("1200x700")
root.configure(bg='black')

style = ttk.Style()
style.theme_use('clam')
style.configure('TFrame', background='black')
style.configure('TButton', 
               background='black', 
               foreground='lime',
               font=('Consolas', 10))
style.map('TButton', 
         background=[('active', 'darkgreen')])

# Frame setup
left_frame = ttk.Frame(root, padding=0)
center_frame = ttk.Frame(root, padding=0)
right_frame = ttk.Frame(root, padding=0)
left_frame.pack(side=tk.LEFT, fill=tk.Y)
center_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
right_frame.pack(side=tk.RIGHT, fill=tk.Y)

# IP Listbox
ip_listbox = Listbox(
    left_frame, 
    bg='black', 
    fg='lime', 
    font=('Consolas', 11),
    selectbackground='darkgreen',
    selectforeground='lime',
    activestyle='none',
    highlightbackground='lime',
    highlightcolor='lime',
    highlightthickness=1
)
ip_listbox.pack(fill=tk.BOTH, expand=True, pady=(0,5))
ip_listbox.bind("<<ListboxSelect>>", lambda e: on_ip_select())

def on_ip_select():
    sel = ip_listbox.curselection()
    if sel:
        selected_ip[0] = ip_listbox.get(sel[0])[2:]  # Remove status symbol
    else:
        selected_ip[0] = None

# Left panel controls
ttk.Button(left_frame, text="Load IPs from File", command=load_ips_from_file
         ).pack(pady=5, fill=tk.X)

# Map initialization
shp_path = filedialog.askdirectory(
    title="Select Shapefile Folder (ne_110m_admin_0_countries)"
)
if not shp_path:
    root.destroy()
    exit()

try:
    # Increase the figsize to make the map canvas larger
    fig, ax = plt.subplots(figsize=(8, 5))  # Adjusted from (8, 5) to (12, 8)
    fig.patch.set_facecolor('black')
    ax.axis('off')
    
    m = Basemap(projection='robin', lat_0=0, lon_0=0, resolution='c', ax=ax)
    m.readshapefile(f'{shp_path}/ne_110m_admin_0_countries', 'countries', drawbounds=False)
    
    create_country_patches()
    
    m.drawcoastlines(color='lime', linewidth=0.5)
    m.drawcountries(color='lime', linewidth=0.2)
    m.fillcontinents(color='dimgray', lake_color='black')
    m.drawmapboundary(fill_color='black')
    
except Exception as e:
    messagebox.showerror("Error", f"Map initialization failed: {str(e)}")
    root.destroy()
    exit()

canvas = FigureCanvasTkAgg(fig, master=center_frame)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
overlay = ax.text(0.02, 0.98, '', color='lime', fontsize=9, 
                transform=ax.transAxes, va='top', zorder=3)

# Event bindings
canvas.mpl_connect('motion_notify_event', hover_handler)
canvas.mpl_connect('scroll_event', zoom)
canvas.mpl_connect('button_press_event', on_press)
canvas.mpl_connect('motion_notify_event', on_drag)
canvas.mpl_connect('button_release_event', on_release)

def invert_icon_colors(icon_path, size_factor=8):  # Increased size_factor from 5 to 8
    """Invert the colors of an icon image and resize it."""
    img = Image.open(icon_path).convert("RGBA")
    inverted_img = ImageOps.invert(img.convert("RGB"))
    inverted_img.putalpha(img.getchannel("A"))  # Preserve transparency
    # Resize the image
    width, height = inverted_img.size
    resized_img = inverted_img.resize((width // size_factor, height // size_factor), Image.Resampling.LANCZOS)
    return ImageTk.PhotoImage(resized_img)

# Load and invert icons for buttons
icon_call_logs = invert_icon_colors("icons/call_logs.png")
icon_sms_inbox = invert_icon_colors("icons/sms_inbox.png")
icon_device_logs = invert_icon_colors("icons/device_logs.png")
icon_location = invert_icon_colors("icons/location.png")
icon_start_scrcpy = invert_icon_colors("icons/start_scrcpy.png")
icon_stop_scrcpy = invert_icon_colors("icons/stop_scrcpy.png")  # Keep red color
icon_refresh = invert_icon_colors("icons/refresh.png")
icon_debug = invert_icon_colors("icons/debug.png")
icon_tcp_mode = invert_icon_colors("icons/tcp_mode.png")
icon_ping_test = invert_icon_colors("icons/ping_test.png")
icon_view_log = invert_icon_colors("icons/view_log.png")

# Arrange buttons in two vertical rows
button_frame_left = ttk.Frame(right_frame, padding=5)
button_frame_right = ttk.Frame(right_frame, padding=5)
button_frame_left.pack(side=tk.LEFT, fill=tk.Y, expand=True)
button_frame_right.pack(side=tk.LEFT, fill=tk.Y, expand=True)

ttk.Button(button_frame_left, image=icon_call_logs, command=lambda: threaded_command("adb shell content query --uri content://call_log/calls")
         ).pack(pady=5, padx=10, fill=tk.X)
ttk.Button(button_frame_left, image=icon_sms_inbox, command=lambda: threaded_command("adb shell content query --uri content://sms/inbox")
         ).pack(pady=5, padx=10, fill=tk.X)
ttk.Button(button_frame_left, image=icon_device_logs, command=lambda: threaded_command("adb logcat -d")
         ).pack(pady=5, padx=10, fill=tk.X)
ttk.Button(button_frame_left, image=icon_location, command=lambda: threaded_command("adb shell dumpsys location")
         ).pack(pady=5, padx=10, fill=tk.X)
ttk.Button(button_frame_left, image=icon_start_scrcpy, command=start_scrcpy
         ).pack(pady=5, padx=10, fill=tk.X)

ttk.Button(button_frame_right, image=icon_stop_scrcpy, command=stop_scrcpy
         ).pack(pady=5, padx=10, fill=tk.X)
ttk.Button(button_frame_right, image=icon_refresh, command=refresh_connections
         ).pack(pady=5, padx=10, fill=tk.X)
ttk.Button(button_frame_right, image=icon_debug, command=debug_connection
         ).pack(pady=5, padx=10, fill=tk.X)
ttk.Button(button_frame_right, image=icon_tcp_mode, command=enable_tcp_mode
         ).pack(pady=5, padx=10, fill=tk.X)
ttk.Button(button_frame_right, image=icon_ping_test, command=raw_ping_test
         ).pack(pady=5, padx=10, fill=tk.X)
ttk.Button(button_frame_right, image=icon_view_log, command=lambda: os.startfile(LOG_FILE)
         ).pack(pady=5, padx=10, fill=tk.X)

test_button = tk.Button(left_frame, text="Test Listbox", command=test_listbox_update)
test_button.pack(pady=5)

# Add a label to display connection stats with an even larger font size
connection_stats_label = ttk.Label(root, text="Online: 0 | Offline: 0", background="black", foreground="lime", font=("Consolas", 18))
button_frame_left.pack(side=tk.BOTTOM, fill=tk.X)

def update_connection_stats():
    """Update the connection stats label with the number of online and offline connections."""
    online_count = len(connected_ips)
    offline_count = len(all_ips) - online_count
    connection_stats_label.config(text=f"Online: {online_count} | Offline: {offline_count}")

# Initial update
update_map_display()
root.mainloop()