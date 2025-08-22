from hal.utilities.image_processing import ImageProcessing
import numpy as np
import cv2
from time import time

class LaneDetect:
    def __init__(self) -> None:
        # Configuración de perspectiva
        self.Source = np.float32([[270, 270], [550, 270], [0, 380], [820, 380]])
        self.Destination = np.float32([[270, 0], [550, 0], [270, 410], [550, 410]])
        
        # Inicialización de variables para líneas
        self.polyright = [0, 0, 0]
        self.polyleft = [0, 0, 0]
        self.left_points = []
        self.right_points = []
        self.polyleft_last = [0, 0, 0]
        self.polyright_last = [0, 0, 0]
        self.error = 0
        self.matrix_perspective = []
        
        # Parámetros para adaptación a cambios de luz
        self.min_hue = 10
        self.max_hue = 45
        self.min_saturation = 50
        self.max_saturation = 255
        self.min_value = 100
        self.max_value = 255
        
        # Parámetros para RANSAC personalizado
        self.ransac_min_samples = 10
        self.ransac_residual_threshold = 2.0
        self.ransac_max_trials = 100
        
        # Cache para la transformación de perspectiva
        self.matrix_cache = {}
        
        # Variables para filtrado temporal
        self.error_history = []
        self.max_history = 5
        
        # Tiempo de procesamiento
        self.process_time = 0

    def adaptive_threshold(self, frame):
        """Aplica umbralización adaptativa basada en condiciones de iluminación"""
        # Convertir a escala de grises
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Calcular iluminación general
        mean_brightness = np.mean(gray)
        
        # Ajustar parámetros basados en el brillo
        if mean_brightness < 80:  # Condiciones oscuras
            self.min_value = max(50, self.min_value - 10)
            self.min_saturation = max(30, self.min_saturation - 10)
        elif mean_brightness > 180:  # Condiciones brillantes
            self.min_value = min(150, self.min_value + 10)
            self.min_saturation = min(80, self.min_saturation + 10)
            
        # Devolver parámetros adaptados
        return np.array([self.min_hue, self.min_saturation, self.min_value]), np.array([self.max_hue, self.max_saturation, self.max_value])

    def TransformImage(self, frame):
        # Redimensionar imagen para mejorar rendimiento
        resize_frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
        original_img = resize_frame.copy()
        
        # Aplicar transformación de perspectiva con caché
        frame_shape = resize_frame.shape
        cache_key = f"{frame_shape[0]}_{frame_shape[1]}"
        
        if cache_key not in self.matrix_cache:
            self.matrix_perspective = cv2.getPerspectiveTransform(self.Source, self.Destination)
            self.matrix_cache[cache_key] = self.matrix_perspective
        else:
            self.matrix_perspective = self.matrix_cache[cache_key]
            
        resize_frame = cv2.warpPerspective(resize_frame, self.matrix_perspective, (resize_frame.shape[1], resize_frame.shape[0]))
        
        # Convertir a HSV y umbralizar para amarillo con parámetros adaptativos
        hsvBuf = cv2.cvtColor(resize_frame, cv2.COLOR_BGR2HSV)
        lower_bounds, upper_bounds = self.adaptive_threshold(resize_frame)
        binaryImage = ImageProcessing.binary_thresholding(frame=hsvBuf, lowerBounds=lower_bounds, upperBounds=upper_bounds)
        
        # Aplicar operaciones morfológicas para mejorar detección
        kernel = np.ones((3, 3), np.uint8)
        binaryImage = cv2.morphologyEx(binaryImage, cv2.MORPH_CLOSE, kernel)
        binaryImage = cv2.morphologyEx(binaryImage, cv2.MORPH_OPEN, kernel)
        
        return original_img, resize_frame, binaryImage
    
    def histogram(self, binaryImage):
        """Implementación optimizada del análisis de histograma"""
        # Utilizar la mitad inferior de la imagen para el histograma
        init_row = binaryImage.shape[0] // 2
        end_row = binaryImage.shape[0] - 1
        roi = binaryImage[init_row:end_row, :]
        
        # Calcular histograma mediante suma por columnas (más rápido)
        histogram = np.sum(roi, axis=0)
        
        # Encontrar los máximos en cada mitad para las líneas izquierda y derecha
        midpoint = binaryImage.shape[1] // 2
        left_ptr = np.argmax(histogram[:midpoint])
        right_ptr = np.argmax(histogram[midpoint:]) + midpoint
        
        # Verificar validez de los máximos
        if histogram[left_ptr] < 50:  # Umbral mínimo
            left_ptr = -1  # Marca como no válido
            
        if histogram[right_ptr] < 50:
            right_ptr = -1
            
        return np.array([left_ptr, right_ptr], dtype=int)
    
    def locate_lanes(self, img):
        """Localiza las líneas de carril usando ventanas deslizantes optimizadas"""
        start_time = time()
        
        # Resetear puntos
        self.left_points = []
        self.right_points = []
        
        # Parámetros de ventanas
        nwindows = 10  # Reducido para mayor velocidad
        margin = 40    # Aumentado para mayor tolerancia
        minpix = 30
        
        # Obtener posiciones iniciales desde histograma
        lane_positions = self.histogram(img)
        leftx_current, rightx_current = lane_positions[0], lane_positions[1]
        
        # Si alguna posición no es válida, usar valores anteriores
        if leftx_current == -1 and hasattr(self, 'left_base_pos'):
            leftx_current = self.left_base_pos
        elif leftx_current != -1:
            self.left_base_pos = leftx_current
            
        if rightx_current == -1 and hasattr(self, 'right_base_pos'):
            rightx_current = self.right_base_pos
        elif rightx_current != -1:
            self.right_base_pos = rightx_current
            
        # Altura de cada ventana
        window_height = img.shape[0] // nwindows
        
        # Encontrar índices de píxeles no cero
        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        
        # Recorrer ventanas
        for window in range(nwindows):
            # Límites de la ventana actual
            win_y_low = img.shape[0] - (window + 1) * window_height
            win_y_high = img.shape[0] - window * window_height
            
            # Límites izquierda y derecha
            win_xleft_low = int(max(0, leftx_current - margin))
            win_xleft_high = int(min(img.shape[1] - 1, leftx_current + margin))
            win_xright_low = int(max(0, rightx_current - margin))
            win_xright_high = int(min(img.shape[1] - 1, rightx_current + margin))
            
            # Identificar píxeles no cero en la ventana (más eficiente con NumPy)
            left_lane_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                             (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            right_lane_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                              (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            
            # Almacenar puntos
            if len(left_lane_inds) > 0:
                for i in left_lane_inds:
                    self.left_points.append((nonzeroy[i], nonzerox[i]))
                leftx_current = int(np.mean(nonzerox[left_lane_inds]))
                
            if len(right_lane_inds) > 0:
                for i in right_lane_inds:
                    self.right_points.append((nonzeroy[i], nonzerox[i]))
                rightx_current = int(np.mean(nonzerox[right_lane_inds]))
        
        self.process_time = time() - start_time
        
    def custom_ransac_fit(self, points, degree=2):
        """Implementación personalizada de RANSAC para ajuste polinomial sin sklearn"""
        if len(points) < self.ransac_min_samples:
            return None, False
            
        # Extraer coordenadas x e y
        x_values = np.array([point[0] for point in points])
        y_values = np.array([point[1] for point in points])
        
        best_coeffs = None
        best_inlier_count = 0
        
        # Ejecutar RANSAC
        for _ in range(self.ransac_max_trials):
            # Seleccionar muestras aleatorias
            sample_indices = np.random.choice(len(points), self.ransac_min_samples, replace=False)
            sample_x = x_values[sample_indices]
            sample_y = y_values[sample_indices]
            
            try:
                # Ajustar polinomio a las muestras
                coeffs = np.polyfit(sample_x, sample_y, degree)
                
                # Calcular errores para todos los puntos
                y_pred = np.polyval(coeffs, x_values)
                errors = np.abs(y_values - y_pred)
                
                # Contar inliers (puntos que se ajustan bien)
                inliers = errors < self.ransac_residual_threshold
                inlier_count = np.sum(inliers)
                
                # Actualizar si se encontró un mejor modelo
                if inlier_count > best_inlier_count:
                    best_inlier_count = inlier_count
                    best_coeffs = coeffs
                    
                    # Salir temprano si encontramos un modelo muy bueno
                    if inlier_count > len(points) * 0.8:
                        break
            except:
                continue
        
        # Si encontramos un buen modelo, refinar con todos los inliers
        if best_coeffs is not None and best_inlier_count >= self.ransac_min_samples:
            # Recalcular usando solo los inliers
            y_pred = np.polyval(best_coeffs, x_values)
            errors = np.abs(y_values - y_pred)
            inliers = errors < self.ransac_residual_threshold
            
            if np.sum(inliers) >= self.ransac_min_samples:
                try:
                    final_coeffs = np.polyfit(x_values[inliers], y_values[inliers], degree)
                    # Coeficientes en orden inverso para compatibilidad con el código existente
                    return final_coeffs, True
                except:
                    pass
            
            return best_coeffs, True
            
        # Si fallamos, intentar con un ajuste normal
        try:
            coeffs = np.polyfit(x_values, y_values, degree)
            return coeffs, True
        except:
            return None, False
    
    def regression_right(self):
        """Ajusta un polinomio a los puntos de línea derecha usando RANSAC personalizado"""
        coeffs, success = self.custom_ransac_fit(self.right_points)
        if success:
            self.polyright = coeffs.tolist()
            return True
        return False

    def regression_left(self):
        """Ajusta un polinomio a los puntos de línea izquierda usando RANSAC personalizado"""
        coeffs, success = self.custom_ransac_fit(self.left_points)
        if success:
            self.polyleft = coeffs.tolist()
            return True
        return False
        
    def calculate_turn_direction(self, error, img_width):
        """Determina la dirección de giro basada en el error y la curvatura"""
        # Normalizar error respecto al ancho de la imagen
        normalized_error = error / (img_width / 2)
        
        # Añadir a historial para suavizado
        self.error_history.append(normalized_error)
        if len(self.error_history) > self.max_history:
            self.error_history.pop(0)
        
        # Calcular error suavizado
        smoothed_error = np.mean(self.error_history)
        
        # Determinar dirección
        if abs(smoothed_error) < 0.1:
            return "CENTRO", smoothed_error
        elif smoothed_error > 0:
            return "IZQUIERDA", smoothed_error
        else:
            return "DERECHA", smoothed_error

    def draw_lines(self, img, original_img):
        """Dibuja las líneas de carril y calcula la dirección de conducción"""
        center_cam = (img.shape[1] // 2) + 22
        center_lines = center_cam  # Valor predeterminado
        
        find_line_right = self.regression_right()
        find_line_left = self.regression_left()
        
        # Limpiar puntos detectados
        self.right_points = []
        self.left_points = []
        
        # Caso 1: Ambas líneas detectadas
        if find_line_left and find_line_right:
            # Dibujar puntos en cada línea
            for row in range(img.shape[0] - 1, -1, -8):
                # Línea derecha
                columnR = self.polyright[2] + self.polyright[1] * row + self.polyright[0] * (row * row)
                cv2.circle(img, (int(columnR), int(row)), 2, (0, 255, 0), 2)
                
                # Línea izquierda
                columnL = self.polyleft[2] + self.polyleft[1] * row + self.polyleft[0] * (row * row)
                cv2.circle(img, (int(columnL), int(row)), 2, (0, 255, 0), 2)
            
            # Calcular centro entre líneas
            center_lines = (columnR + columnL) / 2
            
            # Actualizar polinomios anteriores
            for k in range(3):
                self.polyleft_last[k] = self.polyleft[k]
                self.polyright_last[k] = self.polyright[k]
        
        # Caso 2: Solo línea izquierda
        elif find_line_left:
            for row in range(img.shape[0] - 1, -1, -8):
                columnL = self.polyleft[2] + self.polyleft[1] * row + self.polyleft[0] * (row * row)
                cv2.circle(img, (int(columnL), int(row)), 2, (0, 255, 0), 2)
            
            # Estimar centro (línea izquierda + offset)
            columnL_aux = self.polyleft[2]
            center_lines = columnL_aux + 125
            
            # Actualizar polinomio anterior
            for k in range(3):
                self.polyleft_last[k] = self.polyleft[k]
        
        # Caso 3: Solo línea derecha
        elif find_line_right:
            for row in range(img.shape[0] - 1, -1, -8):
                columnR = self.polyright[2] + self.polyright[1] * row + self.polyright[0] * (row * row)
                cv2.circle(img, (int(columnR), int(row)), 2, (0, 255, 0), 2)
            
            # Estimar centro (línea derecha - offset)
            columnR_aux = self.polyright[2]
            center_lines = columnR_aux - 125
            
            # Actualizar polinomio anterior
            for k in range(3):
                self.polyright_last[k] = self.polyright[k]
        
        # Caso 4: Ninguna línea detectada, usar información anterior
        elif hasattr(self, 'polyleft_last') and hasattr(self, 'polyright_last'):
            for row in range(img.shape[0] - 1, -1, -8):
                # Usar polinomios anteriores
                columnR = self.polyright_last[2] + self.polyright_last[1] * row + self.polyright_last[0] * (row * row)
                cv2.circle(img, (int(columnR), int(row)), 2, (255, 0, 0), 2)  # Rojo para indicar estimación
                
                columnL = self.polyleft_last[2] + self.polyleft_last[1] * row + self.polyleft_last[0] * (row * row)
                cv2.circle(img, (int(columnL), int(row)), 2, (255, 0, 0), 2)
            
            # Calcular centro entre líneas estimadas
            center_lines = (columnR + columnL) / 2
        
        # Asegurar que center_lines está dentro de los límites
        center_lines = min(max(0, center_lines), img.shape[1] - 1)
            
        # Calcular error respecto al centro
        distance_center = center_cam - center_lines
        self.error = distance_center
        
        # Determinar dirección de giro
        turn_direction, smoothed_error = self.calculate_turn_direction(distance_center, img.shape[1])
        
        # Dibujar líneas de referencia
        cv2.line(img, (int(center_cam), int(img.shape[0] / 4)), (int(center_cam), int(img.shape[0] * 3 / 4)), (0, 255, 0), 2)
        cv2.line(img, (int(center_lines), 0), (int(center_cam), int(img.shape[0] - 1)), (0, 0, 255), 2)
        
        # Mostrar dirección y error
        cv2.putText(img, f"Dir: {turn_direction}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(img, f"Error: {smoothed_error:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(img, f"Time: {self.process_time*1000:.1f}ms", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Transformar de vuelta a la perspectiva original
        invertedPerspectiveMatrix = np.linalg.inv(self.matrix_perspective)
        warped_frame = cv2.warpPerspective(img, invertedPerspectiveMatrix, (img.shape[1], img.shape[0]))
        
        # Fusionar con imagen original
        result = cv2.addWeighted(original_img, 0.7, warped_frame, 0.3, 0)
        
        return result
    
    def find_lines(self, frame):
        """Método principal para detectar líneas de carril en una imagen"""
        total_start_time = time()
        
        # Transformar imagen
        original_img, resize_frame, binary_image = self.TransformImage(frame)
        
        # Localizar carriles
        self.locate_lanes(binary_image)
        
        # Dibujar líneas y calcular dirección
        result = self.draw_lines(resize_frame, original_img)
        
        # Calcular tiempo total
        self.process_time = time() - total_start_time
        
        return result