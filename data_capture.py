import os
import subprocess
import datetime
import sqlite3
import customtkinter as ctk

# Configuration
BASE_DIR = os.path.expanduser("~/VR_Interface/usertesting")
DB_PATH = os.path.join(BASE_DIR, 'all_data.db')
TOPICS = [
    "/zed/zed_node/stereo/image_rect_color/compressed",
    "/controller_movement",
    "/franka_robot_state_broadcaster/current_pose",
    "/ft_sensor/data",
    "/new_goal_pose",
    "/rumble_output",
    "/joint_states"
]

class DataCaptureApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("MSI Cubi Robotic Arm Data Capture")
        self.geometry("800x650")

        # State per task
        self.bag_process = {}
        self.record_start = {}
        self.record_stop = {}
        self.session_dir = {}
        self.bag_file = {}

        # UI Variables
        self.participant_id = ctk.StringVar()
        self.knows_robot = ctk.StringVar(value="yes")
        self.notes = {i: ctk.StringVar() for i in range(1,4)}
        self.task_achieved = {i: ctk.StringVar(value="no") for i in range(1,4)}
        self.task_errors = {i: ctk.IntVar(value=0) for i in range(1,4)}
        self.task_restarted = {i: ctk.StringVar(value="no") for i in range(1,4)}
        self.task_restart_count = {i: ctk.IntVar(value=0) for i in range(1,4)}
        self.task_severity = {i: ctk.IntVar(value=0) for i in (2,3)}

        # Ensure base dir exists
        os.makedirs(BASE_DIR, exist_ok=True)
        # Initialize global database
        self._init_database()

        self.build_ui()

    def _init_database(self):
        conn = sqlite3.connect(DB_PATH)
        cur = conn.cursor()
        cur.execute("""
            CREATE TABLE IF NOT EXISTS tasks (
              id INTEGER PRIMARY KEY,
              participant_id TEXT NOT NULL,
              knows_robot TEXT NOT NULL,
              task_number INTEGER NOT NULL,
              rosbag_path TEXT NOT NULL,
              recording_start TEXT NOT NULL,
              recording_stop TEXT NOT NULL,
              recording_duration_seconds INTEGER NOT NULL,
              achieved TEXT NOT NULL,
              errors_flagged INTEGER NOT NULL,
              restarted TEXT NOT NULL,
              restart_count INTEGER NOT NULL,
              error_severity INTEGER,
              notes TEXT
            )
        """)
        conn.commit()
        conn.close()

    def build_ui(self):
        # Participant and group
        top = ctk.CTkFrame(self)
        top.pack(fill="x", padx=20, pady=10)
        ctk.CTkLabel(top, text="Participant ID:").grid(row=0, column=0, sticky="w")
        ctk.CTkEntry(top, textvariable=self.participant_id).grid(row=0, column=1, sticky="w")
        ctk.CTkLabel(top, text="Knows Robot:").grid(row=1, column=0, sticky="w")
        ctk.CTkOptionMenu(top, values=["yes","no"], variable=self.knows_robot).grid(row=1, column=1, sticky="w")

        # Tasks scrollable
        tasks_frame = ctk.CTkScrollableFrame(self, width=760, height=450)
        tasks_frame.pack(padx=20, pady=10)
        for i in range(1,4):
            self._build_task_frame(tasks_frame, i)

        # Save button
        bottom = ctk.CTkFrame(self)
        bottom.pack(fill="x", padx=20, pady=10)
        ctk.CTkButton(bottom, text="Save All Tasks", command=self.save_and_exit).pack(anchor="e")

    def _build_task_frame(self, parent, num):
        f = ctk.CTkFrame(parent)
        f.pack(fill="x", pady=5)
        ctk.CTkLabel(f, text=f"Task {num}", font=ctk.CTkFont(size=16, weight="bold")).grid(row=0, column=0, columnspan=3, sticky="w")

        # Recording controls
        ctk.CTkButton(f, text=f"Start Recording (Task {num})", 
                      command=lambda n=num: self.start_recording(n)).grid(row=1, column=0, pady=5)
        ctk.CTkButton(f, text=f"Stop Recording (Task {num})", 
                      command=lambda n=num: self.stop_recording(n)).grid(row=1, column=1, pady=5)

        # Metrics
        ctk.CTkLabel(f, text="Achieved:").grid(row=2, column=0, sticky="w")
        ctk.CTkOptionMenu(f, values=["yes","no"], variable=self.task_achieved[num]).grid(row=2, column=1, sticky="w")

        ctk.CTkLabel(f, text="# Errors Flagged:").grid(row=3, column=0, sticky="w")
        ctk.CTkSpinBox(f, from_=0, to=100, textvariable=self.task_errors[num], width=80).grid(row=3, column=1, sticky="w")

        ctk.CTkLabel(f, text="Restarted:").grid(row=4, column=0, sticky="w")
        ctk.CTkOptionMenu(f, values=["yes","no"], variable=self.task_restarted[num]).grid(row=4, column=1, sticky="w")

        ctk.CTkLabel(f, text="# Restarts:").grid(row=5, column=0, sticky="w")
        ctk.CTkSpinBox(f, from_=0, to=50, textvariable=self.task_restart_count[num], width=80).grid(row=5, column=1, sticky="w")

        if num in (2,3):
            ctk.CTkLabel(f, text="Error Severity (0/1/3):").grid(row=6, column=0, sticky="w")
            ctk.CTkOptionMenu(f, values=["0","1","3"], variable=self.task_severity[num]).grid(row=6, column=1, sticky="w")

        ctk.CTkLabel(f, text="Notes:").grid(row=7, column=0, sticky="w")
        ctk.CTkEntry(f, textvariable=self.notes[num], width=500).grid(row=7, column=1, columnspan=2, sticky="w")

    def start_recording(self, num):
        pid = self.participant_id.get().strip()
        if not pid:
            ctk.CTkMessageBox(title="Error", message="Participant ID required").show()
            return
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        part_dir = os.path.join(BASE_DIR, pid)
        task_dir = os.path.join(part_dir, f"task_{num}_{ts}")
        os.makedirs(task_dir, exist_ok=True)
        self.session_dir[num] = task_dir

        bag_base = os.path.join(task_dir, f"task_{num}_{ts}")
        cmd = ["ros2", "bag", "record", "-o", bag_base] + TOPICS
        proc = subprocess.Popen(cmd)
        self.bag_process[num] = proc
        self.record_start[num] = datetime.datetime.now().isoformat()

        ctk.CTkMessageBox(title="Recording Started", message=f"Task {num} recording saved to {bag_base}.db3").show()

    def stop_recording(self, num):
        proc = self.bag_process.get(num)
        if not proc:
            ctk.CTkMessageBox(title="Error", message=f"No recording in progress for Task {num}").show()
            return
        proc.terminate()
        proc.wait()
        self.record_stop[num] = datetime.datetime.now().isoformat()
        self.bag_file[num] = os.path.relpath(self.session_dir[num], BASE_DIR) + f".db3"
        ctk.CTkMessageBox(title="Recording Stopped", message=f"Task {num} recording stopped.").show()

    def save_and_exit(self):
        pid = self.participant_id.get().strip()
        # Validate all tasks recorded
        if not pid or any(num not in self.record_stop for num in range(1,4)):
            ctk.CTkMessageBox(title="Error", message="Ensure all 3 tasks are recorded before saving.").show()
            return

        conn = sqlite3.connect(DB_PATH)
        cur = conn.cursor()

        for num in range(1,4):
            start = datetime.datetime.fromisoformat(self.record_start[num])
            stop  = datetime.datetime.fromisoformat(self.record_stop[num])
            dur   = int((stop - start).total_seconds())
            sev   = self.task_severity[num].get() if num in (2,3) else None
            cur.execute(
                "INSERT INTO tasks (participant_id, knows_robot, task_number, rosbag_path, recording_start, recording_stop, recording_duration_seconds, achieved, errors_flagged, restarted, restart_count, error_severity, notes) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)",
                (
                    pid,
                    self.knows_robot.get(),
                    num,
                    self.bag_file[num],
                    self.record_start[num],
                    self.record_stop[num],
                    dur,
                    self.task_achieved[num].get(),
                    self.task_errors[num].get(),
                    self.task_restarted[num].get(),
                    self.task_restart_count[num].get(),
                    sev,
                    self.notes[num].get().strip()
                )
            )
        conn.commit()
        conn.close()

        ctk.CTkMessageBox(title="Saved", message=f"All tasks saved to {DB_PATH}").show()
        self.destroy()

if __name__ == "__main__":
    ctk.set_appearance_mode("System")
    ctk.set_default_color_theme("blue")
    app = DataCaptureApp()
    app.mainloop()
