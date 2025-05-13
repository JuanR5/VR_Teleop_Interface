import os
import sys
import shutil
import sqlite3
import datetime
import subprocess
import tkinter.messagebox as mb
import customtkinter as ctk

if not os.environ.get("DISPLAY"):
    sys.stderr.write("Error: No display. Please run in a graphical environment with $DISPLAY set.\n")
    sys.exit(1)

BASE_DIR = os.path.expanduser("~/VR_Interface/usertesting")
DB_PATH = os.path.join(BASE_DIR, "all_data.db")
BAG_DIR = os.path.join(BASE_DIR, "rosbags")
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
        self.geometry("850x500")

        self.participant_id = ctk.StringVar()
        self.knows_robot = ctk.StringVar(value="yes")
        self.current_task_vars = {}
        self.bag_process = None
        self.record_start = None
        self.record_stop = None
        self.bag_file = None
        self.task_number = None
        self.selected_participant = ctk.StringVar()
        self.selected_task = ctk.StringVar()

        os.makedirs(BASE_DIR, exist_ok=True)
        os.makedirs(BAG_DIR, exist_ok=True)
        self._init_database()
        self._build_ui()

    def _init_database(self):
        with sqlite3.connect(DB_PATH) as conn:
            conn.execute("""
                CREATE TABLE IF NOT EXISTS participants (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    participant_id TEXT UNIQUE NOT NULL,
                    knows_robot TEXT NOT NULL
                )
            """)
            conn.execute("""
                CREATE TABLE IF NOT EXISTS tasks (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    participant_id TEXT NOT NULL,
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
                    notes TEXT,
                    FOREIGN KEY(participant_id) REFERENCES participants(participant_id)
                )
            """)

    def _build_ui(self):
        header = ctk.CTkFrame(self)
        header.pack(fill="x", padx=20, pady=10)

        ctk.CTkLabel(header, text="Participant ID:").grid(row=0, column=0, sticky="w")
        ctk.CTkEntry(header, textvariable=self.participant_id, width=200).grid(row=0, column=1, sticky="w")

        ctk.CTkLabel(header, text="Knows Robot:").grid(row=1, column=0, sticky="w")
        ctk.CTkOptionMenu(header, values=["yes", "no"], variable=self.knows_robot).grid(row=1, column=1, sticky="w")

        self.task_frame = ctk.CTkFrame(self)
        self.task_frame.pack(fill="x", padx=20, pady=10)

        ctk.CTkLabel(self.task_frame, text="TASKS", font=ctk.CTkFont(size=16, weight="bold")).grid(row=0, column=0, sticky="w", pady=(0, 10))
        self.task_label = ctk.CTkLabel(self.task_frame, text="Current Task: Not started")
        self.task_label.grid(row=0, column=1, sticky="w")

        self._create_task_template()
        self._create_dropdowns()

        footer = ctk.CTkFrame(self)
        footer.pack(pady=10)

        self.start_button = ctk.CTkButton(footer, text="Start Recording", command=self._start_recording)
        self.start_button.pack(side="left", padx=10)

        ctk.CTkButton(footer, text="Stop Recording", command=self._stop_recording).pack(side="left", padx=10)
        ctk.CTkButton(footer, text="Save Task", command=self._save_task).pack(side="left", padx=10)
        ctk.CTkButton(footer, text="Delete Selected Task", command=self._delete_task).pack(side="left", padx=10)

    def _create_task_template(self):
        self.unsaved_highlight = []
        self.current_task_vars = {
            'achieved': ctk.StringVar(value="no"),
            'errors': ctk.StringVar(value="0"),
            'restarted': ctk.StringVar(value="no"),
            'restarts': ctk.StringVar(value="0"),
            'severity': ctk.StringVar(value="0"),
            'notes': ctk.StringVar()
        }

        for widget in self.task_frame.winfo_children():
            if isinstance(widget, ctk.CTkLabel) and widget.cget("text") in ["TASKS", "Current Task: Not started"]:
                continue
            widget.destroy()

        fields = [
            ("Achieved:", self.current_task_vars['achieved'], ["yes", "no"]),
            ("# Errors Flagged:", self.current_task_vars['errors'], None),
            ("Restarted:", self.current_task_vars['restarted'], ["yes", "no"]),
            ("# Restarts:", self.current_task_vars['restarts'], None),
            ("Error Severity (0/1/3):", self.current_task_vars['severity'], ["0", "1", "3"]),
            ("Notes:", self.current_task_vars['notes'], None)
        ]

        for i, (label, var, opts) in enumerate(fields):
            ctk.CTkLabel(self.task_frame, text=label).grid(row=i+1, column=0, sticky="w")
            if opts:
                ctk.CTkOptionMenu(self.task_frame, values=opts, variable=var).grid(row=i+1, column=1, sticky="w")
            else:
                entry = ctk.CTkEntry(self.task_frame, textvariable=var, width=500)
                entry.grid(row=i+1, column=1, columnspan=2, sticky="w")
                self.unsaved_highlight.append(entry)

    def _create_dropdowns(self):
        def _load_tasks(participant_id):
            with sqlite3.connect(DB_PATH) as conn:
                cur = conn.cursor()
                cur.execute("SELECT task_number FROM tasks WHERE participant_id = ?", (participant_id,))
                tasks = [str(row[0]) for row in cur.fetchall()]
            self.selected_task.set("-")
            if hasattr(self, 'task_menu'):
                self.task_menu.configure(values=tasks or ["-"])
        self._load_tasks = _load_tasks
        self.dropdown_frame = ctk.CTkFrame(self)
        self.dropdown_frame.pack(fill="x", padx=20, pady=5)

        ctk.CTkLabel(self.dropdown_frame, text="Select Participant:").pack(side="left")
        self.participant_menu = ctk.CTkOptionMenu(self.dropdown_frame, values=["-"], variable=self.selected_participant, command=self._load_tasks)
        self.participant_menu.pack(side="left", padx=10)

        ctk.CTkLabel(self.dropdown_frame, text="Select Task:").pack(side="left")
        self.task_menu = ctk.CTkOptionMenu(self.dropdown_frame, values=["-"], variable=self.selected_task)
        self.task_menu.pack(side="left", padx=10)

        self._update_dropdowns()

    def _update_dropdowns(self):
        with sqlite3.connect(DB_PATH) as conn:
            cur = conn.cursor()
            cur.execute("SELECT DISTINCT participant_id FROM tasks")
            participants = [row[0] for row in cur.fetchall()]
        self.selected_participant.set("-")
        self.selected_task.set("-")
        self.participant_menu.configure(values=participants or ["-"])
        self.task_menu.configure(values=["-"])

    def _delete_task(self):
        pid = self.selected_participant.get()
        task = self.selected_task.get()
        if pid and task and task != "-":
            confirm = mb.askyesno("Confirm Delete", f"Are you sure you want to delete Task {task} for participant {pid}?")
            if not confirm:
                return
            with sqlite3.connect(DB_PATH) as conn:
                cur = conn.cursor()
                cur.execute("SELECT rosbag_path FROM tasks WHERE participant_id = ? AND task_number = ?", (pid, int(task)))
                row = cur.fetchone()
                if row:
                    bag_path = os.path.join(BASE_DIR, row[0]) if not os.path.isabs(row[0]) else row[0]
                    bag_dir = os.path.dirname(bag_path)
                    try:
                        if os.path.isdir(bag_dir):
                            shutil.rmtree(bag_dir)
                    except Exception as e:
                        mb.showwarning("Warning", f"Failed to delete rosbag directory: {e}")
                cur.execute("DELETE FROM tasks WHERE participant_id = ? AND task_number = ?", (pid, int(task)))
            mb.showinfo("Deleted", f"Task {task} for participant {pid} and its rosbag file were deleted.")
            if hasattr(self, "dropdown_frame"):
                self.dropdown_frame.destroy()
            self._create_dropdowns()

    def _start_recording(self):
        if self.bag_process or (self.bag_file and not self.record_stop):
            warn = mb.askyesno(
                "Overwrite Warning",
                "You have an unsaved or unclosed recording. Starting a new one will overwrite its data.\nContinue?"
            )
            if not warn:
                return
            # cleanup the old process if still running
            if self.bag_process:
                self.bag_process.terminate()
                self.bag_process.wait()
            # delete unsaved old rosbag
            if self.bag_dir and os.path.isdir(self.bag_dir):
                try:
                    shutil.rmtree(self.bag_dir)
                except Exception as e:
                    mb.showwarning("Warning", f"Could not remove old rosbag: {e}")
            self.bag_process = None
            self.record_start = None
            self.record_stop = None
            self.bag_file = None

        pid = self.participant_id.get().strip()
        if not pid:
            mb.showerror("Error", "Participant ID required")
            return

        self._init_database()
        with sqlite3.connect(DB_PATH) as conn:
            cur = conn.cursor()
            cur.execute("INSERT OR IGNORE INTO participants (participant_id, knows_robot) VALUES (?, ?)", (pid, self.knows_robot.get()))
            cur.execute("SELECT COUNT(*) FROM tasks WHERE participant_id = ?", (pid,))
            self.task_number = cur.fetchone()[0] + 1

        self.task_label.configure(text=f"Current Task: {self.task_number}")

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_base = os.path.join(BAG_DIR, f"{pid}_task{self.task_number}_{timestamp}")
        self.bag_process = subprocess.Popen(["ros2", "bag", "record", "-o", bag_base] + TOPICS)
        self.record_start = datetime.datetime.now().isoformat()
        self.bag_dir = bag_base  # store the directory
        self.start_button.configure(fg_color="green")
        mb.showinfo("Recording", f"Recording started: {self.bag_dir}")

    def _stop_recording(self):
        if not self.bag_process:
            mb.showerror("Error", "No recording in progress")
            return
        self.bag_process.terminate()
        self.bag_process.wait()
        self.record_stop = datetime.datetime.now().isoformat()
        self.start_button.configure(fg_color="transparent")
        mb.showinfo("Recording", "Recording stopped.")

    def _save_task(self):
        pid = self.participant_id.get().strip()
        if not pid:
            mb.showerror("Error", "Participant ID required")
            return
        if not self.record_start:
            mb.showerror("Error", "No recording has been started for this task")
            return
        if not self.record_stop:
            mb.showerror("Error", "Recording has not been stopped")
            return
            mb.showerror("Error", "Complete recording and participant info first")
            return

        self._init_database()
        with sqlite3.connect(DB_PATH) as conn:
            cur = conn.cursor()
            cur.execute("INSERT OR IGNORE INTO participants (participant_id, knows_robot) VALUES (?, ?)", (pid, self.knows_robot.get()))
            cur.execute("SELECT COUNT(*) FROM tasks WHERE participant_id = ?", (pid,))
            task_num = cur.fetchone()[0] + 1 if self.task_number is None else self.task_number

            start = datetime.datetime.fromisoformat(self.record_start)
            stop = datetime.datetime.fromisoformat(self.record_stop)
            dur = int((stop - start).total_seconds())
            v = self.current_task_vars
            db3_file = next((f for f in os.listdir(self.bag_dir) if f.endswith("_0.db3")), None)
            if not db3_file:
                mb.showerror("Error", "No .db3 file found in recorded rosbag directory")
                return
            self.bag_file = os.path.join(self.bag_dir, db3_file)

            cur.execute("""
                INSERT INTO tasks (participant_id, task_number, rosbag_path, recording_start, recording_stop, recording_duration_seconds, achieved, errors_flagged, restarted, restart_count, error_severity, notes)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                pid, task_num, self.bag_file, self.record_start, self.record_stop, dur,
                v['achieved'].get(), int(v['errors'].get()), v['restarted'].get(),
                int(v['restarts'].get()), int(v['severity'].get()), v['notes'].get().strip()
            ))

        for entry in self.unsaved_highlight:
            entry.configure(border_color="green")
        self._reset_task_form()
        mb.showinfo("Saved", f"Task {task_num} for participant {pid} saved.")

    def _reset_task_form(self):
        self.record_start = None
        self.record_stop = None
        self.bag_file = None
        self.bag_process = None
        self.task_number = None
        self.participant_id.set("")
        self.knows_robot.set("yes")
        self.start_button.configure(fg_color="transparent")
        self.task_label.configure(text="Current Task: Not started")
        self._create_task_template()
        if hasattr(self, "dropdown_frame"):
            self.dropdown_frame.destroy()
        self._create_dropdowns()

if __name__ == "__main__":
    ctk.set_appearance_mode("System")
    ctk.set_default_color_theme("blue")
    app = DataCaptureApp()
    app.mainloop()
