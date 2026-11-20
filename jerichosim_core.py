# Создаём базовую структуру симулятора
# py E:\Jericho\private_use\tasks\3D-симулятор\jerichosim_core.py

import numpy as np
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any
import json
import time

# Попробуем импортировать pybullet
try:
    import pybullet as p
    import pybullet_data
    BULLET_AVAILABLE = True
except ImportError:
    BULLET_AVAILABLE = False
    print("⚠️ PyBullet не установлен. Установите: pip install pybullet")

# ---------- Абстрактный бэкенд ----------
class SimulatorBackend:
    """Абстрактный интерфейс к физическому движку."""
    
    def load_scene(self, path: str) -> bool:
        raise NotImplementedError
    
    def get_lidar(self, num_rays: int = 360, max_distance: float = 10.0) -> np.ndarray:
        raise NotImplementedError
    
    def get_camera(self, width: int = 320, height: int = 240) -> np.ndarray:
        raise NotImplementedError
    
    def get_audio_text(self) -> List[str]:
        raise NotImplementedError
    
    def apply_action(self, action: Dict[str, Any]) -> None:
        raise NotImplementedError
    
    def step(self) -> None:
        raise NotImplementedError
    
    def get_state(self) -> Dict[str, Any]:
        raise NotImplementedError

# ---------- PyBullet реализация ----------
if BULLET_AVAILABLE:
    class PyBulletBackend(SimulatorBackend):
        def __init__(self, scene_path: str = None):
            self.client_id = p.connect(p.DIRECT)  # или p.GUI для визуализации
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0, 0, -9.81)
            p.setTimeStep(1./240.)
            
            self.robot_id = None
            self.camera_id = None
            self.audio_events = []
            self.step_counter = 0
            
            if scene_path:
                self.load_scene(scene_path)
            else:
                # Загружаем тестовую сцену по умолчанию
                p.loadURDF("plane.urdf")
                p.loadURDF("table/table.urdf", [0, 0, 0])
                p.loadURDF("r2d2.urdf", [0.5, 0, 0.6])
        
        def load_scene(self, path: str) -> bool:
            """Загружает сцену из файла (URDF, OBJ, SDF)."""
            try:
                if path.endswith('.urdf'):
                    self.robot_id = p.loadURDF(path)
                elif path.endswith('.obj'):
                    # Для OBJ нужен коллайдер
                    collision_id = p.createCollisionShape(p.GEOM_MESH, fileName=path)
                    visual_id = p.createVisualShape(p.GEOM_MESH, fileName=path)
                    self.robot_id = p.createMultiBody(0, collision_id, visual_id)
                else:
                    return False
                return True
            except Exception as e:
                print(f"Ошибка загрузки сцены: {e}")
                return False
        
        def get_lidar(self, num_rays: int = 360, max_distance: float = 10.0) -> np.ndarray:
            """Возвращает массив расстояний в метрах для каждого луча."""
            if self.robot_id is None:
                return np.full(num_rays, max_distance)
            
            pos, orn = p.getBasePositionAndOrientation(self.robot_id)
            euler = p.getEulerFromQuaternion(orn)
            yaw = euler[2]
            
            distances = np.full(num_rays, max_distance)
            for i in range(num_rays):
                angle = yaw + (2 * np.pi * i / num_rays)
                direction = [np.cos(angle), np.sin(angle), 0]
                to_pos = [pos[j] + direction[j] * max_distance for j in range(3)]
                
                ray = p.rayTest(pos, to_pos)
                if ray[0][0] != -1:  # есть попадание
                    hit_fraction = ray[0][2]
                    distances[i] = hit_fraction * max_distance
            
            return distances
        
        def get_camera(self, width: int = 320, height: int = 240) -> np.ndarray:
            """Возвращает RGB-изображение с камеры робота."""
            if self.robot_id is None:
                return np.zeros((height, width, 3), dtype=np.uint8)
            
            pos, orn = p.getBasePositionAndOrientation(self.robot_id)
            rot_matrix = p.getMatrixFromQuaternion(orn)
            
            # Камера направлена вперёд
            forward = [rot_matrix[0], rot_matrix[3], rot_matrix[6]]
            target = [pos[i] + forward[i] * 5.0 for i in range(3)]
            up = [0, 0, 1]
            
            view_matrix = p.computeViewMatrix(pos, target, up)
            projection_matrix = p.computeProjectionMatrixFOV(
                fov=60, aspect=width/height, nearVal=0.1, farVal=100
            )
            
            img = p.getCameraImage(width, height, view_matrix, projection_matrix)
            rgb = np.array(img[2], dtype=np.uint8).reshape(height, width, 4)[:, :, :3]
            return rgb
        
        def get_audio_text(self) -> List[str]:
            """Возвращает текстовые события 'звука' с последнего шага."""
            events = self.audio_events.copy()
            self.audio_events.clear()
            return events
        
        def apply_action(self, action: Dict[str, Any]) -> None:
            """Применяет действие к роботу."""
            if self.robot_id is None:
                return
            
            action_type = action.get('action', 'stop')
            if action_type == 'forward':
                speed = action.get('speed', 1.0)
                # Движение вперёд по направлению робота
                pos, orn = p.getBasePositionAndOrientation(self.robot_id)
                rot_matrix = p.getMatrixFromQuaternion(orn)
                forward = [rot_matrix[0], rot_matrix[3], rot_matrix[6]]
                new_pos = [pos[i] + forward[i] * speed * (1./240.) for i in range(3)]
                p.resetBasePositionAndOrientation(self.robot_id, new_pos, orn)
            
            elif action_type == 'turn':
                angle = action.get('angle', 0.1)
                pos, orn = p.getBasePositionAndOrientation(self.robot_id)
                euler = p.getEulerFromQuaternion(orn)
                new_orn = p.getQuaternionFromEuler([euler[0], euler[1], euler[2] + angle])
                p.resetBasePositionAndOrientation(self.robot_id, pos, new_orn)
            
            elif action_type == 'stop':
                pass  # ничего не делаем
        
        def step(self) -> None:
            """Шаг симуляции."""
            p.stepSimulation()
            self.step_counter += 1
        
        def get_state(self) -> Dict[str, Any]:
            """Возвращает полное состояние для агента."""
            lidar = self.get_lidar().tolist()
            camera = self.get_camera()
            audio = self.get_audio_text()
            
            return {
                'step': self.step_counter,
                'lidar': lidar,
                'camera_shape': camera.shape,
                'audio_events': audio,
                'timestamp': time.time()
            }
        
        def close(self):
            p.disconnect(self.client_id)

# ---------- Менеджер симуляции ----------
class SimulationManager:
    """Управляет циклом симуляции и связью с агентом."""
    
    def __init__(self, backend: SimulatorBackend, agent=None):
        self.backend = backend
        self.agent = agent
        self.running = False
    
    def step_with_agent(self) -> Dict[str, Any]:
        """Один шаг: сенсоры → агент → действие."""
        state = self.backend.get_state()
        
        if self.agent:
            # Здесь будет вызов агента Лин
            action = self.agent.decide(state)
            self.backend.apply_action(action)
        
        self.backend.step()
        return state
    
    def run(self, steps: int = 1000):
        """Запускает симуляцию на заданное число шагов."""
        self.running = True
        for _ in range(steps):
            if not self.running:
                break
            state = self.step_with_agent()
            # Здесь можно отправлять state агенту через API
        self.running = False

# ---------- Тест ----------
if __name__ == "__main__":
    if BULLET_AVAILABLE:
        print("✅ PyBullet доступен. Создаю тестовую сцену...")
        backend = PyBulletBackend()
        manager = SimulationManager(backend)
        
        for i in range(100):
            state = backend.get_state()
            if i % 10 == 0:
                print(f"Шаг {state['step']}, лидар[0]: {state['lidar'][0]:.2f} м")
            backend.step()
        
        backend.close()
        print("✅ Тест пройден!")
    else:
        print("❌ Установите PyBullet: pip install pybullet")