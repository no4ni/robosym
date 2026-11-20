import pybullet as p
import pybullet_data
import time
import json
import os
from pathlib import Path
import numpy as np
import threading
import queue
import win32gui
import win32con
import win32api
import cv2
import math

class JerichoSimulator:
    def __init__(self):
        floor_z = -1.0  # Фикс
        # Основной клиент с GUI (физика + визуал)
        self.client = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)

        
        p.resetDebugVisualizerCamera(
            cameraDistance=2.5,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, -0.7]
        )


        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1./240.)
        self.cam_distance = 2
        self.cam_yaw = 50
        self.cam_pitch = -30

        # --- ГРУЗИМ КОЛЛИЗИЮ ПЕРВЫМ ДЕЛОМ ---
        self.scene_dir = Path(os.environ.get("SIM_SCENE_DIR") or Path(__file__).resolve().parent)
        self.rot_quat = p.getQuaternionFromEuler([-math.radians(90), -math.radians(-180), 0])
        print("📦 Загружаю тяжелую коллизию (пол и стены)...")
        p.setAdditionalSearchPath(str(self.scene_dir))
        col_path = "hatacol.obj"
        try:
            _cid = p.createCollisionShape(p.GEOM_MESH, fileName=col_path, meshScale=[1,1,1], flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
            # Невидимый visual-shape, чтобы pybullet не рисовал collision как жёлтый каркас
            _vid_dummy = p.createVisualShape(p.GEOM_SPHERE, radius=0.0001, rgbaColor=[0,0,0,0])
            self.apartment_collision = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=_cid, baseVisualShapeIndex=_vid_dummy)
            p.resetBasePositionAndOrientation(self.apartment_collision, [8.0, -22.0, -1.0], self.rot_quat)
            print("✅ Коллизия успешно загружена! Робот не провалится.")
        except Exception as e:
            print(f"⚠️ Ошибка коллизии: {e}")
        
        # 1. Стандартные модели (робот)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # Старт — в центре квартиры (не в туалете)
        self.start_pos = [5.6, 0.0, -0.6]
        self.robot = p.loadURDF("r2d2.urdf", self.start_pos, globalScaling=0.3)
        p.resetDebugVisualizerCamera(
            cameraDistance=2.5, cameraYaw=45, cameraPitch=-30,
            cameraTargetPosition=self.start_pos
        )
        
        # 2. Загрузка квартиры (визуал, коллизия, пол)
        rot_quat = p.getQuaternionFromEuler([-math.radians(90), -math.radians(-180), 0])
        objects_dir = self.scene_dir / "objects_opt"
        print(f"🏠 Загружаю квартиру из: {objects_dir}")
        p.setAdditionalSearchPath(str(self.scene_dir))
        
        if objects_dir.exists():
            import glob as _glob
            obj_files = list(objects_dir.glob("*.obj"))
            print(f"🏠 Загружаю {len(obj_files)} объектов из папки objects...")
            self.apartment_bodies = []
            self.fib_prev = 1
            self.fib_curr = 1
            self.load_delay = 1
            self.obj_queue = [f for f in obj_files if not ("Default_material" in f.name or "Cube" in f.name or "Material" in f.name)]
            self.rot_quat = p.getQuaternionFromEuler([-math.radians(90), -math.radians(-180), 0])
            print(f"📦 Подготовлено {len(self.obj_queue)} объектов для постепенной загрузки...")
        else:
            print("⚠️ Папка objects не найдена!")

        collision_path = self.scene_dir / "hatacol.obj"
        # Простой пол-бокс вместо 48-МБ коллизии
        import pybullet_data as _pd
        p.setAdditionalSearchPath(_pd.getDataPath())
        floor_z = -1.01
        _fh = 50.0
        _ft = 0.2
        _fs = p.createCollisionShape(p.GEOM_BOX, halfExtents=[_fh, _fh, _ft/2])
        _fv = p.createVisualShape(p.GEOM_BOX, halfExtents=[_fh, _fh, _ft/2], rgbaColor=[0.6, 0.6, 0.6, 1])
        self.floor = p.createMultiBody(0, _fs, _fv, basePosition=[0, 0, floor_z - _ft/2])
        print(f"✅ Плоская поверхность создана на высоте {floor_z} м")
        
        # 3. Отдельный клиент DIRECT для чистой камеры
        self.client_cam = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client_cam)
        p.setAdditionalSearchPath(str(self.scene_dir), physicsClientId=self.client_cam)
        
        # Robot View: грузим те же 95 объектов, что и GUI (одна картинка)
        self.apartment_cam_bodies = []
        _obj_dir_cam = self.scene_dir / "objects_opt"
        if _obj_dir_cam.exists():
            _count = 0
            for _obj_file in _obj_dir_cam.glob("*.obj"):
                try:
                    _vid = p.createVisualShape(
                        p.GEOM_MESH,
                        fileName=str(_obj_file).replace("\\", "/"),
                        meshScale=[1,1,1],
                        physicsClientId=self.client_cam
                    )
                    _vbody = p.createMultiBody(0, -1, _vid, physicsClientId=self.client_cam)
                    p.resetBasePositionAndOrientation(_vbody, [8.0, -22.0, -1.0], self.rot_quat, physicsClientId=self.client_cam)
                    self.apartment_cam_bodies.append(_vbody)
                    _count += 1
                except Exception as e:
                    print(f"cam obj fail {_obj_file.name}: {e}", flush=True)
            print(f"📷 Robot View: загружено {_count} объектов", flush=True)
        else:
            print(f"⚠️ Robot View: не найдено {_obj_dir_cam}", flush=True)
        
        r2d2_path = os.path.join(pybullet_data.getDataPath(), "r2d2.urdf")
        self.robot_cam = p.loadURDF(r2d2_path, [0, 0, floor_z + 0.5], globalScaling=0.3, useFixedBase=False, physicsClientId=self.client_cam)
        p.setGravity(0,0,0, physicsClientId=self.client_cam)
        
        self.robot_view_positioned = False
        self.update_overview_camera()
        p.resetBasePositionAndOrientation(self.robot_cam, self.start_pos, [0, 0, 0, 1], physicsClientId=self.client_cam)
        self.show_robot_view()
        
        # Интерфейс и консоль
        _cmd_env = os.environ.get("SIM_CMD_FILE")
        if _cmd_env:
            self.command_file = Path(_cmd_env)
        else:
            self.command_file = self.scene_dir / "command.json"
        self.running = True
        self.command_queue = queue.Queue()
        self.step_counter = 0
        self.console_thread = threading.Thread(target=self.console_reader, daemon=True)
        self.console_thread.start()
        print("✅ Симулятор запущен! Команды: fwd 0.2 | turn 0.5 | reset | quit", flush=True)
        
        time.sleep(0.5)
        hwnd = win32gui.FindWindow(None, "Bullet Physics ExampleBrowser using OpenGL3+ [btgl] Release build")
        if hwnd:
            screen_width = win32api.GetSystemMetrics(0)
            win_width = 300
            win_height = 600
            x = screen_width - win_width
            y = 0
            win32gui.SetWindowPos(hwnd, win32con.HWND_TOPMOST, x, y, win_width, win_height, win32con.SWP_SHOWWINDOW)

    def console_reader(self):
        while self.running:
            try:
                cmd = input()
                if cmd.strip():
                    self.command_queue.put(cmd.strip())
            except EOFError:
                break

    def get_lidar(self, num_rays=7, max_dist=10.0):
        pos, orn = p.getBasePositionAndOrientation(self.robot)
        start_pos = [pos[0], pos[1], pos[2] + 0.3]
        euler = p.getEulerFromQuaternion(orn)
        yaw = euler[2]
        angles = np.linspace(-np.pi/3, np.pi/3, num_rays) + yaw + np.pi/2
        distances = np.full(num_rays, max_dist)
        for i, angle in enumerate(angles):
            dx = np.cos(angle)
            dy = np.sin(angle)
            to = [start_pos[0] + dx * max_dist, start_pos[1] + dy * max_dist, start_pos[2]]
            ray = p.rayTest(start_pos, to)
            if ray[0][0] != -1:
                distances[i] = ray[0][2] * max_dist
        return angles, distances

    def format_lidar(self, angles, distances):
        yaw = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.robot)[1])[2]
        rel_angles = [(a - yaw + np.pi) % (2*np.pi) - np.pi for a in angles]
        sorted_pairs = sorted(zip(rel_angles, distances), key=lambda x: x[0])
        parts = []
        for angle_rad, dist in sorted_pairs:
            angle_deg = round(np.degrees(angle_rad))
            parts.append(f"{angle_deg}°:{dist:.2f}м")
        return " | ".join(parts)

    def update_overview_camera(self, desired=1.8, cam_pitch=-25):
        pos, _ = p.getBasePositionAndOrientation(self.robot, physicsClientId=self.client)
        _target = [pos[0], pos[1], pos[2] + 0.2]
        _pr = math.radians(cam_pitch)
        best_yaw, best_dist = 45, 0.6
        for _y in range(0, 360, 30):
            _yr = math.radians(_y)
            _dir = [math.cos(_pr) * math.cos(_yr), math.cos(_pr) * math.sin(_yr), math.sin(_pr)]
            _to = [_target[i] + _dir[i] * desired for i in range(3)]
            _hit = p.rayTest(_target, _to)[0]
            if _hit[0] == -1 or _hit[0] == self.robot:
                _free = desired
            else:
                _free = max(0.5, desired * _hit[2] * 0.85)
            if _free > best_dist:
                best_dist = _free
                best_yaw = _y
            if _free >= desired * 0.95:
                best_yaw = _y
                best_dist = desired
                break
        p.resetDebugVisualizerCamera(
            cameraDistance=best_dist,
            cameraYaw=best_yaw,
            cameraPitch=cam_pitch,
            cameraTargetPosition=_target
        )

    def show_robot_view(self):
        pos, orn = p.getBasePositionAndOrientation(self.robot, physicsClientId=self.client)
        p.resetBasePositionAndOrientation(self.robot_cam, pos, orn, physicsClientId=self.client_cam)
        
        # Камера от "головы" робота, горизонтальный взгляд по yaw
        # (игнорируем roll/pitch — иначе при падении камера заваливается)
        euler = p.getEulerFromQuaternion(orn)
        yaw = euler[2]
        eye = [pos[0], pos[1], pos[2] + 0.15]
        target = [eye[0] - math.sin(yaw) * 3.0, eye[1] + math.cos(yaw) * 3.0, eye[2]]
        print(f"📷 cam: pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) yaw={math.degrees(yaw):.1f}° eye=({eye[0]:.2f},{eye[1]:.2f},{eye[2]:.2f}) tgt=({target[0]:.2f},{target[1]:.2f},{target[2]:.2f})", flush=True)
        view = p.computeViewMatrix(eye, target, [0, 0, 1])
        proj = p.computeProjectionMatrixFOV(60, 4/3, 0.1, 100)
        w, h = 320, 240
        img = p.getCameraImage(w, h, view, proj, physicsClientId=self.client_cam)
        rgb = np.array(img[2], dtype=np.uint8).reshape(h, w, 4)[:, :, :3]
        
        cv2.imshow("Robot View (Camera RGB)", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        hwnd = win32gui.FindWindow(None, "Robot View (Camera RGB)")
        if hwnd:
            win32gui.SetWindowPos(hwnd, win32con.HWND_TOPMOST, 0, 0, 0, 0, win32con.SWP_NOMOVE | win32con.SWP_NOSIZE)
            if not self.robot_view_positioned:
                screen_height = win32api.GetSystemMetrics(1)
                win32gui.SetWindowPos(hwnd, win32con.HWND_TOPMOST, 0, 0, 320, 240, win32con.SWP_SHOWWINDOW)
                self.robot_view_positioned = True
        cv2.waitKey(1)
        
    def apply_action(self, action):
        action_type = action.get("action", "")
        pos, orn = p.getBasePositionAndOrientation(self.robot)
        euler = p.getEulerFromQuaternion(orn)
        
        if action_type == "forward":
            speed = action.get("speed", 0.1)
            STEP = 0.05  # 5 см — не перепрыгиваем стены
            steps = max(1, int(abs(speed) / STEP))
            step_size = speed / steps
            for _ in range(steps):
                pos, orn = p.getBasePositionAndOrientation(self.robot)
                euler = p.getEulerFromQuaternion(orn)
                yaw = euler[2]
                new_pos = [pos[0] - np.sin(yaw) * step_size, pos[1] + np.cos(yaw) * step_size, pos[2]]
                p.resetBasePositionAndOrientation(self.robot, new_pos, orn)
                p.stepSimulation()
                # контакт со стенами/мебелью => откат и стоп
                walls = p.getContactPoints(bodyA=self.robot, bodyB=self.apartment_collision)
                if walls:
                    p.resetBasePositionAndOrientation(self.robot, pos, orn)
                    print(f"🧱 стена/мебель, откат на шаге {_+1}/{steps}", flush=True)
                    break
        elif action_type == "turn":
            angle = action.get("angle", 0.1)
            new_orn = p.getQuaternionFromEuler([0, 0, euler[2] - angle])
            p.resetBasePositionAndOrientation(self.robot, pos, new_orn)
        elif action_type == "reset":
            p.resetBasePositionAndOrientation(self.robot, self.start_pos, [0, 0, 0, 1])
            p.resetDebugVisualizerCamera(
                cameraDistance=2.5, cameraYaw=45, cameraPitch=-30,
                cameraTargetPosition=self.start_pos
            )
        
        # После движения обновляем камеру, чтобы она следовала за роботом
        pos, orn = p.getBasePositionAndOrientation(self.robot, physicsClientId=self.client)
        self.update_overview_camera()
        # После получения pos, orn:
        euler = p.getEulerFromQuaternion(orn)
        print(f"🤖 POS=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})  RPY=({math.degrees(euler[0]):.1f}°, {math.degrees(euler[1]):.1f}°, {math.degrees(euler[2]):.1f}°)", flush=True)
        yaw_deg = math.degrees(euler[2])
        compass = (yaw_deg + 360) % 360
        print(f"🧭 Компас: {compass:.1f}°")
        angles, distances = self.get_lidar()
        print(f"📡 Лидар: {self.format_lidar(angles, distances)}")
        print(f"📍 GPS: {57.164544+pos[0]*0.000009:.6f}, {65.503498+pos[1]*0.000009:.6f}") 
        try:
            import os as _os, json as _json
            _scene = _os.environ.get("SIM_SCENE_DIR") or str(self.scene_dir)
            _tel_path = _os.path.join(_scene, "telemetry.json")
            _tel_data = {"lidar": self.format_lidar(angles, distances), "gps": f"{57.164544+pos[0]*0.000009:.6f}, {65.503498+pos[1]*0.000009:.6f}", "compass": compass}
            with open(_tel_path, "w", encoding="utf-8") as _tf:
                _json.dump(_tel_data, _tf, ensure_ascii=False)
        except Exception as e:
            print("Telemetry save error:", e)
        p.resetBasePositionAndOrientation(self.robot_cam, pos, orn, physicsClientId=self.client_cam)
        self.show_robot_view()
        return action_type

    def parse_console_command(self, cmd_str):
        if cmd_str.startswith('{'):
            try:
                return json.loads(cmd_str)
            except:
                return None
        parts = cmd_str.split()
        if not parts:
            return None
        cmd = parts[0].lower()
        if cmd in ["quit", "exit"]:
            self.running = False
            return None
        if cmd == "reset":
            return {"action": "reset"}
        if cmd in ["fwd", "forward"]:
            speed = float(parts[1]) if len(parts) > 1 else 0.1
            return {"action": "forward", "speed": speed}
        if cmd == "turn":
            angle = float(parts[1]) if len(parts) > 1 else 0.1
            return {"action": "turn", "angle": angle}
        return None

    def run(self):
        while self.running:
            # 1) команды из файла (SIM_CMD_FILE / command.json) — пишет sim.py
            if self.command_file.exists():
                try:
                    _raw = self.command_file.read_text(encoding="utf-8")
                    self.command_file.unlink()
                    _data = json.loads(_raw)
                    if _data:
                        self.apply_action(_data)
                except Exception as e:
                    print(f"⚠️ cmd file: {e}", flush=True)
            # 2) команды из консоли
            try:
                cmd_str = self.command_queue.get_nowait()
                cmd = self.parse_console_command(cmd_str)
                if cmd:
                    self.apply_action(cmd)
            except queue.Empty:
                pass
            
            p.stepSimulation()
            try:
                cv2.waitKey(1)  # поддерживаем окно Robot View живым
            except Exception:
                pass
            
            # Загрузка объектов (по одному за кадр, но это не блокирует камеру)
            if hasattr(self, 'obj_queue') and self.obj_queue:
                self.load_delay -= 1
                if self.load_delay <= 0:
                    _obj = self.obj_queue.pop(0)
                    try:
                        _vid = p.createVisualShape(p.GEOM_MESH, fileName=str(_obj).replace("\\", "/"), meshScale=[1,1,1])
                        _vbody = p.createMultiBody(0, -1, _vid)
                        self.apartment_bodies.append(_vbody)
                        p.resetBasePositionAndOrientation(_vbody, [8.0, -22.0, -1.0], self.rot_quat)
                    except Exception as e:
                        print(f"Ошибка загрузки {_obj.name}: {e}")
                    # Константная задержка: 1 объект за кадр (было Fibonacci — слишком медленно)
                    self.load_delay = 1
            
            
            time.sleep(1./240.)

if __name__ == "__main__":
    sim = JerichoSimulator()
    sim.run()